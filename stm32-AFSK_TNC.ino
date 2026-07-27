/*
 * STM32 Blue Pill AFSK 1200 TNC - Dual Mode (KISS + Command)
 * =========================================================
 * Self-contained AX.25 / Bell-202 (1200 baud) modem. No external AFSK
 * library required -- everything (modulation, demodulation, HDLC, CRC)
 * is implemented below.
 *
 * TOOLCHAIN
 *   Arduino IDE / arduino-cli with the OFFICIAL STMicroelectronics core:
 *     Boards Manager URL:
 *       https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
 *     Board:  "Generic STM32F1 series" -> "BluePill F103C8" (or C8T6)
 *     USB support: "CDC (generic Serial supersede U(S)ART)" if you want
 *                  Serial over the native USB port; otherwise use USART1.
 *
 * HARDWARE (STM32F103C8T6)
 *   RX Audio -> PA0 (ADC).  AC-couple the radio's speaker/discriminator
 *               audio through ~0.1uF and bias PA0 to 1.65V with a
 *               2x10k divider from 3V3 to GND. Keep peak-to-peak well
 *               under 3.3V (a series pot helps).
 *   TX Audio -> PA1 (PWM). RC low-pass to the radio mic:
 *               PA1 --[ 4.7k ]--+--> mic, and 10nF from that node to GND
 *               (fc ~= 3.4 kHz). Add a series cap + trimmer to set level.
 *   PTT      -> PA2 -> NPN base (1k), collector pulls the radio PTT to GND.
 *   Status   -> PC13 onboard LED (active LOW). Steady = OK/idle;
 *               100 ms blink = serial/on-air activity; 500 ms blink = connected.
 *
 * NOTES ON RELIABILITY
 *   The demodulator is the classic MicroModem/BeRTOS delay-multiply +
 *   digital-PLL design and decodes real on-air 1200 packets, but AFSK RX
 *   is analog-sensitive: audio level, DC bias and the RC filter all matter.
 *   For this baseline the ADC is read with analogRead() inside the sample
 *   ISR for portability. If decode is marginal, the single biggest upgrade
 *   is to trigger the ADC directly from the timer and read ADC1->DR (removes
 *   sampling jitter). See the comment in sampleISR().
 */

#include <Arduino.h>
#include <HardwareTimer.h>
#include <math.h>

// ---------------- GPS serial port ----------------
// Two ways to talk to the GPS; pick with the flag below.
//
//   GPS_USE_SOFTWARE_SERIAL 1  (default): bit-banged SoftwareSerial on PB11/PB10.
//     Compiles on any board configuration. The modem's continuous 9600 Hz ISR
//     competes with its bit timing, so some NMEA sentences may be dropped - bad
//     ones fail the checksum and are discarded, so you still get a fix, just a
//     little slower. Perfectly usable for beaconing.
//
//   GPS_USE_SOFTWARE_SERIAL 0: hardware USART3 via the core's Serial3 instance.
//     On this core you CANNOT declare your own HardwareSerial object (the Arduino.h
//     chain only exposes the abstract base class), so we use the predefined Serial3.
//     TWO things are required, both external to this file:
//       1) Tools > "U(S)ART support" = "Enabled (generic 'Serial')" (not "Disabled").
//       2) A file named  build_opt.h  MUST sit in this sketch's folder containing:
//              -DENABLE_HWSERIAL3 -DPIN_SERIAL3_RX=PB11 -DPIN_SERIAL3_TX=PB10
//          That is what actually creates the Serial3 object on USART3. Without it
//          you get "undefined reference to `Serial3'" at link time.
//     Your USB-CDC console stays on `Serial`. USART3 pins: PB10 = TX, PB11 = RX.
#define GPS_USE_SOFTWARE_SERIAL 0
#define GPS_RX_PIN PB11   // GPS TX -> here (USART3 RX)
#define GPS_TX_PIN PB10   // GPS RX <- here (USART3 TX, usually unused)
#if GPS_USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
#else
  // The core expects Serial3 to exist but failed to compile it.
  // We manually define it here using the correct 'Uart' class to satisfy the linker!
  Uart Serial3(GPS_RX_PIN, GPS_TX_PIN);
  // Map our gpsSerial reference to this newly created Serial3 object
  #define gpsSerial Serial3
#endif

// APRS symbol for GPS position beacons (runtime-settable via the SYMBOL command).
// Table id: '/' primary, '\' alternate, or an overlay char (0-9 / A-Z).
// Code examples: '>' car, '-' house, 'b' bicycle, '[' jogger, 'k' truck, '_' wx.
// Symbol codes are CASE-SENSITIVE.
char aprsSymTable = '/';
char aprsSymCode  = '>';

// ---------------- Pin map ----------------
#define PTT_PIN PA2
#define TX_PIN  PA1   // TIM2_CH2
#define RX_PIN  PA0   // ADC_IN0
#define LED_PIN         PC13  // Blue Pill onboard LED (active LOW)
#define LED_ACTIVE_LOW  1     // set 0 if you wire an external active-high LED

// ---------------- Modem constants ----------------
#define SAMPLERATE      9600            // ADC / DDS sample rate (Hz)
#define BITRATE         1200
#define SAMPLESPERBIT   (SAMPLERATE / BITRATE)   // = 8
#define MARK_FREQ       1200
#define SPACE_FREQ      2200

// DDS (numerically controlled oscillator) for TX
#define SIN_LEN         256             // MUST stay 256 (uint8 phase auto-wraps)
#define PWM_TOP         255             // 8-bit PWM (ARR = 255 -> ~281 kHz carrier)
#define TX_AMPLITUDE    110             // sine swing around mid-scale (keep < 128)
static uint8_t sineTable[SIN_LEN];
// Phase increment per sample = SIN_LEN * freq / SAMPLERATE
#define MARK_INC        ((uint8_t)((uint32_t)SIN_LEN * MARK_FREQ  / SAMPLERATE))  // 32
#define SPACE_INC       ((uint8_t)((uint32_t)SIN_LEN * SPACE_FREQ / SAMPLERATE))  // 59

// Digital-PLL bit sampler (values from the reference MicroModem design)
#define PHASE_BITS      8
#define PHASE_INC       3
#define PHASE_MAX       (SAMPLESPERBIT * PHASE_BITS)   // 64
#define PHASE_THRES     (PHASE_MAX / 2)                // 32

// TX framing
#define TX_PREAMBLE_FLAGS  32    // ~213 ms of 0x7E flags (in addition to PTT lead)
#define TX_TAIL_FLAGS      3

// Buffers
#define MAX_FRAME       340             // AX.25 frame incl. FCS
#define TX_BITBUF_BYTES  700            // packed HDLC bitstream (~5600 bits)

// ---------------- Timers ----------------
HardwareTimer *PwmTimer;      // TIM2 -> PWM on PA1
HardwareTimer *SampleTimer;   // TIM3 -> sample-rate ISR

// ---------------- Shared / ISR state ----------------
volatile bool transmitting = false;
volatile bool txDone       = false;

// TX bitstream (built in main context, consumed by ISR)
static uint8_t  txBitBuf[TX_BITBUF_BYTES];
static uint32_t txBitCount = 0;
static uint8_t  txStuffOnes = 0;
volatile uint32_t txBitIndex = 0;
static uint8_t  txSampleInBit = 0;
static uint8_t  txPhase = 0;
static uint8_t  txPhaseInc = MARK_INC;
static bool     txMark = true;          // current NRZI tone

// RX demodulator state
static int16_t iirX[2], iirY[2];
static int8_t  delayLine[SAMPLESPERBIT / 2];   // 4-sample delay for discriminator
static uint8_t delayPos = 0;
static uint8_t sampledBits = 0;         // raw tone decisions (at ADC rate)
static int8_t  currentPhase = 0;
static uint8_t actualBits = 0;          // decisions at bit rate

// HDLC receive assembly
static uint8_t  hdlcBits = 0;
static uint8_t  rxByte = 0;
static uint8_t  rxBitCount = 0;
static bool     rxReceiving = false;
static uint8_t  rxFrameBuf[MAX_FRAME];  // assembled in ISR
static uint16_t rxFrameLen = 0;

// Handoff from ISR to main loop
static uint8_t           rxFrame[MAX_FRAME];   // copy for main loop
volatile uint16_t        rxRawLen = 0;
volatile bool            rxFrameReady = false;

// ---------------- TNC operating modes ----------------
enum TNCMode { COMMAND, CONVERSE, KISS };
TNCMode currentMode = COMMAND;

// KISS protocol bytes
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

uint8_t  kissBuffer[330];
uint16_t kissBufferIndex = 0;
bool     escapeNext = false;
bool     inKissFrame = false;

char    inputBuffer[150];
uint8_t inputLen = 0;

// TNC configuration
char    myCall[7]   = "NOCALL";
uint8_t mySSID      = 0;
char    destCall[7] = "APRS  ";
uint8_t destSSID    = 0;

// Beacon
char     beaconText[100] = "STM32 TNC";
bool     beaconEnabled   = false;
uint32_t beaconInterval  = 600000UL;    // ms (default 10 min)
uint32_t lastBeaconMs    = 0;

// GPS (NMEA decoder)
bool     gpsEnabled   = false;
uint32_t gpsBaud      = 9600;
bool     gpsFixValid  = false;
char     gpsLatRaw[12] = "";            // NMEA ddmm.mmmm
char     gpsLonRaw[12] = "";            // NMEA dddmm.mmmm
char     gpsLatHemi   = 'N';
char     gpsLonHemi   = 'W';
uint8_t  gpsSats      = 0;
char     gpsAlt[10]   = "";             // metres
char     gpsUtc[12]   = "";             // hhmmss.ss
uint32_t gpsLastFixMs = 0;
char     nmea[100];
uint8_t  nmeaLen = 0;

// Digipeater (New N-Paradigm)
enum DigiMode { DIGI_FILL, DIGI_WIDE };
bool     digiEnabled  = false;
DigiMode digiMode     = DIGI_FILL;      // FILL = WIDE1-1 only (home / fill-in)
uint8_t  wideMax      = 2;              // max N handled in WIDE mode (anti-flood)
char     myAlias[7]   = "";             // optional extra exact-match alias
uint8_t  myAliasSSID  = 0;
bool     aliasEnabled = false;

// Duplicate suppression: drop packets whose content we repeated recently
#define DEDUPE_SIZE  16
#define DEDUPE_MS    30000UL
uint16_t dupeSig[DEDUPE_SIZE];
uint32_t dupeAt[DEDUPE_SIZE];
uint8_t  dupeIdx = 0;

// ---------------- Status LED ----------------
#define  ACTIVITY_HOLD_MS 250       // fast-blink window after activity (ms)
uint32_t lastActivityMs = 0;
bool     linkConnected  = false;    // drives the 500 ms "connected" blink
uint32_t ledLastToggle  = 0;
bool     ledState       = true;

// Outgoing digipeater path (used by beacon / converse / text TX)
#define MAX_PATH 8
char    txPathCalls[MAX_PATH][7];
uint8_t txPathSSID[MAX_PATH];
uint8_t txPathCount = 0;

// ============================================================
//  Status LED
// ============================================================
void ledWrite(bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(LED_PIN, on ? LOW : HIGH);
#else
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}

// Call whenever serial or on-air traffic occurs.
void noteActivity(void) { lastActivityMs = millis(); }

// Non-blocking LED state machine (call every loop):
//   CONNECTED       -> blink 500 ms   (highest priority)
//   recent activity -> blink 100 ms   (serial or on-air)
//   otherwise (OK)  -> steady ON
void updateLED(void) {
  uint32_t now = millis();
  uint16_t period;
  if (linkConnected)                                            period = 500;
  else if ((uint32_t)(now - lastActivityMs) < ACTIVITY_HOLD_MS) period = 100;
  else                                                          period = 0;

  if (period == 0) {
    if (!ledState) { ledState = true; ledWrite(true); }   // steady ON
  } else if ((uint32_t)(now - ledLastToggle) >= period) {
    ledLastToggle = now;
    ledState = !ledState;
    ledWrite(ledState);
  }
}

// ============================================================
//  CRC-16 / X.25 (AX.25 FCS)  poly 0x8408 (reflected), init 0xFFFF
// ============================================================
static uint16_t crcUpdate(uint16_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x0001) crc = (crc >> 1) ^ 0x8408;
    else              crc >>= 1;
  }
  return crc;
}

static bool checkFCS(uint8_t *buf, uint16_t len) {
  if (len < 2) return false;
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len - 2; i++) crc = crcUpdate(crc, buf[i]);
  crc ^= 0xFFFF;                                   // ones-complement
  return ((uint8_t)(crc & 0xFF) == buf[len - 2]) &&
         ((uint8_t)(crc >> 8)  == buf[len - 1]);
}

// ============================================================
//  TX: build the HDLC bitstream (flags + stuffed data + FCS)
// ============================================================
static inline void txPushBitRaw(uint8_t bit) {
  if ((txBitCount >> 3) >= TX_BITBUF_BYTES) return;  // overflow guard
  uint32_t idx = txBitCount >> 3;
  uint8_t  msk = 1 << (txBitCount & 7);
  if (bit) txBitBuf[idx] |=  msk;
  else     txBitBuf[idx] &= ~msk;
  txBitCount++;
}

static inline void txPushDataBit(uint8_t bit) {   // with bit-stuffing
  txPushBitRaw(bit);
  if (bit) {
    if (++txStuffOnes == 5) { txPushBitRaw(0); txStuffOnes = 0; }
  } else {
    txStuffOnes = 0;
  }
}

static void txPushByte(uint8_t b) {               // LSB first, stuffed
  for (uint8_t i = 0; i < 8; i++) txPushDataBit((b >> i) & 1);
}

static void txPushFlag(void) {                    // 0x7E, NOT stuffed
  txStuffOnes = 0;
  txPushBitRaw(0);
  for (uint8_t i = 0; i < 6; i++) txPushBitRaw(1);
  txPushBitRaw(0);
}

// PTT control (radio needs time to key up / PLL lock)
void triggerPTT(bool state) {
  digitalWrite(PTT_PIN, state ? HIGH : LOW);
  if (state) delay(250);   // TX delay: let the transmitter come up
  else       delay(50);    // TX tail
}

// Take a raw AX.25 frame (address..info, NO FCS), append FCS,
// HDLC-encode and key the transmitter.
void sendAX25Frame(uint8_t *data, uint16_t len) {
  if (len == 0 || len > (MAX_FRAME - 2)) return;
  noteActivity();                    // on-air TX activity

  // 1) FCS over the frame data
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) crc = crcUpdate(crc, data[i]);
  crc ^= 0xFFFF;
  uint8_t fcsLo = crc & 0xFF;
  uint8_t fcsHi = (crc >> 8) & 0xFF;

  // 2) Build the bitstream
  txBitCount = 0;
  txStuffOnes = 0;
  for (uint16_t i = 0; i < TX_PREAMBLE_FLAGS; i++) txPushFlag();
  for (uint16_t i = 0; i < len; i++)               txPushByte(data[i]);
  txPushByte(fcsLo);
  txPushByte(fcsHi);
  for (uint16_t i = 0; i < TX_TAIL_FLAGS; i++)     txPushFlag();

  // 3) Fire the DDS via the sample ISR
  txBitIndex = 0;
  txSampleInBit = 0;
  txPhase = 0;
  txMark = true;
  txPhaseInc = MARK_INC;
  txDone = false;

  triggerPTT(true);
  transmitting = true;               // ISR now generates tones
  while (!txDone) { /* wait for the whole frame to clock out */ }
  transmitting = false;

  PwmTimer->setCaptureCompare(2, PWM_TOP / 2, TICK_COMPARE_FORMAT); // silence
  triggerPTT(false);
}

// Encode one AX.25 address (callsign + SSID). Sets the extension bit on the
// final address of the field and leaves the H ("has-been-repeated") bit clear.
void encodeAddress(const char *call, uint8_t ssid, uint8_t *out, bool isLast) {
  uint8_t n = strlen(call);
  for (uint8_t i = 0; i < 6; i++)
    out[i] = ((i < n) ? call[i] : ' ') << 1;
  out[6] = ((ssid & 0x0F) << 1) | 0x60;        // reserved bits set
  if (isLast) out[6] |= 0x01;                  // extension bit -> last address
}

// Build a UI frame (dest, source, optional TX path, ctrl, PID, payload) and TX.
void buildAndSendUI(const char *payload, uint16_t len) {
  uint8_t frame[MAX_FRAME];
  uint16_t p = 0;
  encodeAddress(destCall, destSSID, &frame[p], false);            p += 7;
  encodeAddress(myCall,   mySSID,   &frame[p], txPathCount == 0); p += 7;
  for (uint8_t i = 0; i < txPathCount; i++) {
    encodeAddress(txPathCalls[i], txPathSSID[i], &frame[p], i == txPathCount - 1);
    p += 7;
  }
  frame[p++] = 0x03;   // UI control
  frame[p++] = 0xF0;   // PID: no layer-3
  for (uint16_t i = 0; i < len && p < MAX_FRAME - 2; i++) frame[p++] = payload[i];
  sendAX25Frame(frame, p);
}

// Convenience: transmit typed text as a UI frame (converse mode)
void transmitTextPacket(char *payload, uint8_t len) {
  buildAndSendUI(payload, len);
  Serial.println("\r\n[TX OK]");
}

// Send the configured beacon once
// Convert an NMEA coordinate (ddmm.mmmm / dddmm.mmmm) + hemisphere into the
// APRS fixed-width form ddmm.mmN / dddmm.mmW (2 decimal minutes).
void aprsCoord(const char *raw, char hemi, char *out) {
  int dot = -1;
  for (int i = 0; raw[i]; i++) if (raw[i] == '.') { dot = i; break; }
  if (dot < 0) { out[0] = '\0'; return; }
  int end = dot + 3;                       // integer part + '.' + 2 decimals
  int oi = 0;
  for (int k = 0; k < end && raw[k]; k++) out[oi++] = raw[k];
  out[oi++] = hemi;
  out[oi] = '\0';
}

// Build the beacon payload. With a valid GPS fix -> APRS position + BTEXT comment;
// otherwise the plain BTEXT (status) string, preserving the old behaviour.
void buildBeaconPayload(char *out, int outSize) {
  if (gpsEnabled && gpsFixValid) {
    char lat[12], lon[13];
    aprsCoord(gpsLatRaw, gpsLatHemi, lat);
    aprsCoord(gpsLonRaw, gpsLonHemi, lon);
    snprintf(out, outSize, "!%s%c%s%c%s", lat, aprsSymTable, lon, aprsSymCode, beaconText);
  } else {
    strncpy(out, beaconText, outSize - 1);
    out[outSize - 1] = '\0';
  }
}

void sendBeacon(void) {
  char payload[160];
  buildBeaconPayload(payload, sizeof(payload));
  buildAndSendUI(payload, strlen(payload));
  Serial.print("\r\n[BEACON sent] ");
  Serial.println(payload);
  if (currentMode == COMMAND) Serial.print("cmd: ");
}

// ---- Digipeater ------------------------------------------------------------
// Does a 7-byte AX.25 address equal call+ssid?
bool addrMatches(const uint8_t *addr, const char *call, uint8_t ssid) {
  uint8_t n = strlen(call);
  for (uint8_t i = 0; i < 6; i++) {
    char c = (char)((addr[i] >> 1) & 0x7F);
    char e = (i < n) ? call[i] : ' ';
    if (c != e) return false;
  }
  return (((addr[6] >> 1) & 0x0F) == (ssid & 0x0F));
}

// Insert a 7-byte address at byte offset `off`, shifting the rest right.
// Returns false if there is no room. `hbit` sets the has-been-repeated flag.
bool insertAddress(uint8_t *frame, uint16_t *len, uint16_t off,
                   const char *call, uint8_t ssid, bool hbit) {
  if (*len + 7 > (MAX_FRAME - 2)) return false;
  memmove(frame + off + 7, frame + off, *len - off);
  encodeAddress(call, ssid, &frame[off], false);   // inserted addr is never last
  if (hbit) frame[off + 6] |= 0x80;
  *len += 7;
  return true;
}

// Content signature for duplicate detection: dest + source + info, ignoring the
// via path and the C/H bits, so an original and all of its repeats hash alike.
uint16_t packetSignature(uint8_t *frame, uint16_t len) {
  uint16_t i = 0; bool end = false;
  while (i + 7 <= len) { if (frame[i + 6] & 0x01) { end = true; break; } i += 7; }
  if (!end) return 0;
  uint16_t infoStart = i + 7;
  uint16_t crc = 0xFFFF;
  for (uint8_t k = 0;  k < 6;  k++)  crc = crcUpdate(crc, frame[k]);     // dest call
  for (uint8_t k = 7;  k < 13; k++)  crc = crcUpdate(crc, frame[k]);     // src call
  crc = crcUpdate(crc, (uint8_t)((frame[13] >> 1) & 0x0F));              // src ssid
  for (uint16_t k = infoStart; k < len; k++) crc = crcUpdate(crc, frame[k]);
  return crc;
}

bool seenRecently(uint16_t sig) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < DEDUPE_SIZE; i++)
    if (dupeAt[i] && dupeSig[i] == sig && (uint32_t)(now - dupeAt[i]) < DEDUPE_MS)
      return true;
  return false;
}

void rememberPacket(uint16_t sig) {
  dupeSig[dupeIdx] = sig;
  dupeAt[dupeIdx]  = millis();
  dupeIdx = (dupeIdx + 1) % DEDUPE_SIZE;
}

// Validate a WIDEn-N address. On success returns n and N (the SSID hop count).
bool isWideN(const uint8_t *addr, uint8_t *n, uint8_t *N) {
  char c[6];
  for (uint8_t k = 0; k < 6; k++) c[k] = (char)((addr[k] >> 1) & 0x7F);
  if (!(c[0] == 'W' && c[1] == 'I' && c[2] == 'D' && c[3] == 'E' &&
        c[4] >= '1' && c[4] <= '7' && c[5] == ' '))
    return false;
  *n = c[4] - '0';
  *N = (addr[6] >> 1) & 0x0F;
  return (*N >= 1 && *N <= *n);        // N must not exceed n
}

// New N-Paradigm digipeater. Acts on the FIRST not-yet-repeated hop:
//   - exact MYCALL match      -> mark used, retransmit
//   - exact plain-alias match -> substitute MYCALL*, retransmit
//   - WIDEn-N, N == 1         -> substitute MYCALL*, retransmit
//   - WIDEn-N, N  > 1         -> decrement to WIDEn-(N-1), insert MYCALL* ahead
// FILL mode handles WIDE1-1 only; WIDE mode handles WIDEn-N up to wideMax.
// A 30 s content cache suppresses duplicates and breaks loops.
void tryDigipeat(uint8_t *frame, uint16_t len) {
  if (!digiEnabled) return;

  // Map the address field (dest, source, up to 8 digis).
  int addrCount = 0; uint16_t i = 0; bool foundEnd = false;
  while (i + 7 <= len && addrCount < 2 + 8) {
    addrCount++;
    if (frame[i + 6] & 0x01) { foundEnd = true; break; }   // extension bit
    i += 7;
  }
  if (!foundEnd || addrCount < 3) return;             // need dest+source+>=1 digi
  if (addrMatches(&frame[7], myCall, mySSID)) return; // never repeat ourselves

  // First hop that has NOT been repeated yet.
  int slot = -1;
  for (int a = 2; a < addrCount; a++)
    if (!(frame[a * 7 + 6] & 0x80)) { slot = a; break; }
  if (slot < 0) return;                               // path exhausted, not ours

  uint16_t off     = (uint16_t)slot * 7;
  bool     wasLast = frame[off + 6] & 0x01;
  uint8_t  digis   = addrCount - 2;
  bool     doTx    = false;

  if (addrMatches(&frame[off], myCall, mySSID)) {              // routed to us
    frame[off + 6] |= 0x80;
    doTx = true;
  } else if (aliasEnabled && addrMatches(&frame[off], myAlias, myAliasSSID)) {
    encodeAddress(myCall, mySSID, &frame[off], wasLast);       // alias -> our call
    frame[off + 6] |= 0x80;
    doTx = true;
  } else {
    uint8_t n, N;
    if (isWideN(&frame[off], &n, &N)) {
      bool ok = (digiMode == DIGI_FILL) ? (n == 1 && N == 1)
                                        : (N <= wideMax);
      if (ok) {
        if (N == 1) {                                          // final hop
          encodeAddress(myCall, mySSID, &frame[off], wasLast);
          frame[off + 6] |= 0x80;
        } else {                                               // hops remain
          frame[off + 6] = (((N - 1) & 0x0F) << 1) | 0x60 | (wasLast ? 0x01 : 0);
          if (digis < 8) insertAddress(frame, &len, off, myCall, mySSID, true);
        }
        doTx = true;
      }
    }
  }
  if (!doTx) return;

  uint16_t sig = packetSignature(frame, len);
  if (sig && seenRecently(sig)) return;               // duplicate -> drop
  if (sig) rememberPacket(sig);

  delay(random(60, 260));                             // brief anti-collision slot
  sendAX25Frame(frame, len);
  Serial.println("\r\n[DIGI repeated]");
  if (currentMode == COMMAND) Serial.print("cmd: ");
}

// ---- Command helpers -------------------------------------------------------
// Parse "CALL" or "CALL-SSID" into a 6-char callsign + SSID.
void parseCallSSID(String token, char *callOut, uint8_t *ssidOut) {
  token.trim();
  int dash = token.indexOf('-');
  String c = (dash >= 0) ? token.substring(0, dash) : token;
  uint8_t s = (dash >= 0) ? (uint8_t)token.substring(dash + 1).toInt() : 0;
  c.trim();
  memset(callOut, 0, 7);
  c.toCharArray(callOut, 7);
  *ssidOut = s & 0x0F;
}

// Parse a comma-separated path list, e.g. "WIDE1-1,WIDE2-1".
void setTxPath(String list) {
  list.trim();
  txPathCount = 0;
  int start = 0;
  while (start < (int)list.length() && txPathCount < MAX_PATH) {
    int comma = list.indexOf(',', start);
    String tok = (comma >= 0) ? list.substring(start, comma) : list.substring(start);
    tok.trim();
    if (tok.length() > 0) {
      parseCallSSID(tok, txPathCalls[txPathCount], &txPathSSID[txPathCount]);
      txPathCount++;
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  Serial.print("TX path set ("); Serial.print(txPathCount); Serial.println(" hops).");
}

// ============================================================
//  AX.25 v2.2 Connected Mode  (single link, mod-8)
// ============================================================
enum LinkState { LINK_DISC, LINK_SETUP, LINK_CONN, LINK_RELEASE };
LinkState linkState = LINK_DISC;

char    peerCall[7] = "";
uint8_t peerSSID = 0;

// Path used for this connection (rebuilt into every frame we send)
char    connPathCall[MAX_PATH][7];
uint8_t connPathSSID[MAX_PATH];
uint8_t connPathCount = 0;

// Sequence state (mod 8)
#define AX_MOD     8
#define AX_WINDOW  4
uint8_t v_s = 0, v_r = 0, v_a = 0;
bool    peerBusy = false;
bool    rejSent  = false;

// Timers / retries (runtime-adjustable via FRACK / RETRY commands)
#define T1_MS_DEF   4000UL    // default retransmit timeout (FRACK)
#define T3_MS       180000UL  // idle link-check timer
#define N2_DEF      10        // default max retries
uint32_t frackMs  = T1_MS_DEF;
uint8_t  retryMax = N2_DEF;
uint32_t t1At = 0; bool t1On = false;
uint32_t t3At = 0; bool t3On = false;
uint8_t  n2 = 0;

// Unacked I-frames (indexed by N(S)) + outbound segment queue.
// PACLEN_MAX is the fixed buffer ceiling; paclen is the runtime limit.
#define PACLEN_MAX 255
#define PACLEN_DEF 128
uint8_t paclen = PACLEN_DEF;
struct Seg { uint8_t data[PACLEN_MAX]; uint8_t len; };
Seg     iStore[AX_MOD];
Seg     txQ[AX_MOD];
uint8_t txQHead = 0, txQTail = 0, txQCount = 0;

// Control-field bases (P/F bit is 0x10)
#define U_SABM 0x2F
#define U_DISC 0x43
#define U_DM   0x0F
#define U_UA   0x63
#define U_FRMR 0x87
#define U_UI   0x03
#define S_RR   0x01
#define S_RNR  0x05
#define S_REJ  0x09
#define PF_BIT 0x10

bool monitorOn = true;

// MHEARD list
#define MHEARD_SIZE 8
char     heardCall[MHEARD_SIZE][10];
uint32_t heardAt[MHEARD_SIZE];
uint8_t  heardCount = 0;

static inline uint8_t seqAdd(uint8_t a, uint8_t b) { return (a + b) & (AX_MOD - 1); }
static inline uint8_t outstanding(void)            { return (v_s - v_a) & (AX_MOD - 1); }

// Is N(R) within the window (v_a .. v_s inclusive)?
bool nrValid(uint8_t nr) {
  uint8_t d1 = (nr  - v_a) & (AX_MOD - 1);
  uint8_t d2 = (v_s - v_a) & (AX_MOD - 1);
  return d1 <= d2;
}

void t1Start(void) { t1At = millis(); t1On = true; }
void t1Stop(void)  { t1On = false; }
void t3Start(void) { t3At = millis(); t3On = true; }
void t3Stop(void)  { t3On = false; }

void setPeer(const char *c, uint8_t s) { memset(peerCall, 0, 7); strncpy(peerCall, c, 6); peerSSID = s; }
bool samePeer(const char *c, uint8_t s) { return (strcmp(peerCall, c) == 0 && peerSSID == s); }
void printPeer(void) { Serial.print(peerCall); if (peerSSID) { Serial.print("-"); Serial.print(peerSSID); } }

// Store the reverse of a received frame's digipeater path for our replies.
void storeReversePath(uint8_t *frame, int addrCount) {
  connPathCount = 0;
  for (int a = addrCount - 1; a >= 2 && connPathCount < MAX_PATH; a--) {
    uint16_t off = (uint16_t)a * 7;
    int l = 6;
    for (int k = 0; k < 6; k++) connPathCall[connPathCount][k] = (char)((frame[off + k] >> 1) & 0x7F);
    while (l > 0 && connPathCall[connPathCount][l - 1] == ' ') l--;
    connPathCall[connPathCount][l] = '\0';
    connPathSSID[connPathCount] = (frame[off + 6] >> 1) & 0x0F;
    connPathCount++;
  }
}

// Build dest=peer, src=me, + path, with command/response C bits. Returns length.
uint16_t connHeader(uint8_t *f, bool isCommand) {
  encodeAddress(peerCall, peerSSID, &f[0], false);
  encodeAddress(myCall,   mySSID,   &f[7], connPathCount == 0);
  if (isCommand) { f[6] |= 0x80; f[13] &= ~0x80; }   // command:  dest C=1, src C=0
  else           { f[6] &= ~0x80; f[13] |= 0x80; }   // response: dest C=0, src C=1
  uint16_t p = 14;
  for (uint8_t i = 0; i < connPathCount; i++) {
    encodeAddress(connPathCall[i], connPathSSID[i], &f[p], i == connPathCount - 1);
    p += 7;
  }
  return p;
}

void sendU(uint8_t type, bool isCommand, bool pf) {
  uint8_t f[80];
  uint16_t p = connHeader(f, isCommand);
  f[p++] = type | (pf ? PF_BIT : 0);
  sendAX25Frame(f, p);
}

void sendS(uint8_t type, bool isCommand, bool pf) {
  uint8_t f[80];
  uint16_t p = connHeader(f, isCommand);
  f[p++] = (v_r << 5) | (pf ? PF_BIT : 0) | type;
  sendAX25Frame(f, p);
}

void sendIFromStore(uint8_t ns, bool pf) {
  uint8_t f[80 + PACLEN_MAX];
  uint16_t p = connHeader(f, true);                        // I-frames are commands
  f[p++] = (v_r << 5) | (pf ? PF_BIT : 0) | (ns << 1);     // I control (bit0 = 0)
  f[p++] = 0xF0;                                           // PID: no layer-3
  for (uint8_t i = 0; i < iStore[ns].len; i++) f[p++] = iStore[ns].data[i];
  sendAX25Frame(f, p);
}

// Acknowledge everything up to nr-1.
void ackUpTo(uint8_t nr) {
  if (!nrValid(nr)) return;
  while (v_a != nr) v_a = seqAdd(v_a, 1);
  n2 = 0;
  if (v_a == v_s) t1Stop(); else t1Start();
}

void retransmitFrom(uint8_t from) {
  uint8_t i = from;
  while (i != v_s) { sendIFromStore(i, false); i = seqAdd(i, 1); }
  t1Start();
}

// Turn queued segments into I-frames while the window has room.
void pumpTx(void) {
  while (txQCount > 0 && outstanding() < AX_WINDOW && !peerBusy && linkState == LINK_CONN) {
    iStore[v_s] = txQ[txQHead];
    txQHead = seqAdd(txQHead, 1);
    txQCount--;
    sendIFromStore(v_s, false);
    v_s = seqAdd(v_s, 1);
    if (!t1On) t1Start();
  }
}

bool enqueueData(const uint8_t *d, uint8_t len) {
  if (txQCount >= AX_MOD) return false;
  Seg *s = &txQ[txQTail];
  s->len = (len > paclen) ? paclen : len;
  memcpy(s->data, d, s->len);
  txQTail = seqAdd(txQTail, 1);
  txQCount++;
  return true;
}

void connReset(void) {
  linkState = LINK_DISC;
  linkConnected = false;                    // LED: back to idle
  t1Stop(); t3Stop();
  v_s = v_r = v_a = 0; n2 = 0; peerBusy = false; rejSent = false;
  txQHead = txQTail = txQCount = 0;
}

void enterConnConverse(void) {
  currentMode = CONVERSE;
  Serial.println("(connected - type to send, Ctrl+C for command mode)");
}

void linkFailure(const char *why) {
  Serial.print("\r\n*** LINK FAILURE: "); Serial.println(why);
  connReset();
  if (currentMode == CONVERSE) currentMode = COMMAND;
  Serial.print("cmd: ");
}

// Record a heard station (from the source address) for MHEARD.
void recordHeard(uint8_t *frame) {
  char cs[10]; int n = 0;
  for (int k = 0; k < 6; k++) { char c = (char)((frame[7 + k] >> 1) & 0x7F); if (c != ' ') cs[n++] = c; }
  uint8_t ss = (frame[13] >> 1) & 0x0F;
  if (ss) { cs[n++] = '-'; if (ss >= 10) { cs[n++] = '1'; cs[n++] = '0' + (ss - 10); } else cs[n++] = '0' + ss; }
  cs[n] = '\0';
  for (uint8_t i = 0; i < heardCount; i++) if (strcmp(heardCall[i], cs) == 0) { heardAt[i] = millis(); return; }
  if (heardCount < MHEARD_SIZE) { strcpy(heardCall[heardCount], cs); heardAt[heardCount] = millis(); heardCount++; }
  else {
    uint8_t oldest = 0;
    for (uint8_t i = 1; i < MHEARD_SIZE; i++) if (heardAt[i] < heardAt[oldest]) oldest = i;
    strcpy(heardCall[oldest], cs); heardAt[oldest] = millis();
  }
}

// Parse and dispatch a received frame (terminal / connected mode).
void ax25HandleRx(uint8_t *frame, uint16_t len) {
  int addrCount = 0; uint16_t i = 0; bool end = false;
  while (i + 7 <= len && addrCount < 2 + MAX_PATH) {
    addrCount++;
    if (frame[i + 6] & 0x01) { end = true; break; }
    i += 7;
  }
  if (!end) return;
  uint16_t ctrlOff = i + 7;
  if (ctrlOff >= len) return;

  uint8_t control = frame[ctrlOff];
  bool    pf      = control & PF_BIT;

  char srcCall[7];
  int sl = 6;
  for (int k = 0; k < 6; k++) srcCall[k] = (char)((frame[7 + k] >> 1) & 0x7F);
  while (sl > 0 && srcCall[sl - 1] == ' ') sl--;
  srcCall[sl] = '\0';
  uint8_t srcSSID = (frame[13] >> 1) & 0x0F;

  bool destUs = addrMatches(&frame[0], myCall, mySSID);
  bool destC  = frame[6]  & 0x80;
  bool srcC   = frame[13] & 0x80;
  bool isCmd  = destC && !srcC;

  // ---- UI frame: monitor display ----
  if ((control & ~PF_BIT) == U_UI) {
    if (monitorOn) {
      uint16_t infoOff = ctrlOff + 2;                 // control + PID
      Serial.print("\r\n<UI "); Serial.print(srcCall);
      if (srcSSID) { Serial.print("-"); Serial.print(srcSSID); }
      Serial.print("> ");
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print("\r\ncmd: "); else Serial.print("\r\n");
    }
    return;
  }

  if (!destUs) return;                                // link frames must target us
  if (addrMatches(&frame[7], myCall, mySSID)) return; // ignore our own echoes

  // ---- U frames ----
  if ((control & 0x03) == 0x03) {
    uint8_t u = control & ~PF_BIT;
    if (u == U_SABM) {
      if (linkState == LINK_CONN && !samePeer(srcCall, srcSSID)) return;   // busy
      setPeer(srcCall, srcSSID);
      storeReversePath(frame, addrCount);
      v_s = v_r = v_a = 0; n2 = 0; peerBusy = false; rejSent = false;
      txQHead = txQTail = txQCount = 0;
      linkState = LINK_CONN;
      linkConnected = true;                 // LED: connected
      sendU(U_UA, false, pf);
      t1Stop(); t3Start();
      Serial.print("\r\n*** CONNECTED to "); printPeer(); Serial.println();
      enterConnConverse();
    } else if (u == U_DISC) {
      sendU(U_UA, false, pf);
      if (linkState == LINK_CONN) Serial.print("\r\n*** DISCONNECTED\r\n");
      connReset();
      if (currentMode == CONVERSE) currentMode = COMMAND;
      Serial.print("cmd: ");
    } else if (u == U_UA) {
      if (linkState == LINK_SETUP) {
        linkState = LINK_CONN; v_s = v_r = v_a = 0; n2 = 0; t1Stop(); t3Start();
        linkConnected = true;               // LED: connected
        Serial.print("\r\n*** CONNECTED to "); printPeer(); Serial.println();
        enterConnConverse();
      } else if (linkState == LINK_RELEASE) {
        Serial.print("\r\n*** DISCONNECTED\r\n"); connReset(); Serial.print("cmd: ");
      }
    } else if (u == U_DM) {
      if (linkState == LINK_SETUP)      { Serial.print("\r\n*** BUSY (connect refused)\r\n"); connReset(); Serial.print("cmd: "); }
      else if (linkState == LINK_CONN)  { Serial.print("\r\n*** DISCONNECTED (by peer)\r\n"); connReset(); if (currentMode == CONVERSE) currentMode = COMMAND; Serial.print("cmd: "); }
    } else if (u == U_FRMR) {
      if (linkState == LINK_CONN) { Serial.print("\r\n*** FRMR - resetting link\r\n"); sendU(U_DISC, true, true); linkState = LINK_RELEASE; n2 = 0; t1Start(); }
    }
    return;
  }

  // Not in a connected session: reject stray S/I with DM.
  if (linkState != LINK_CONN) {
    if (linkState == LINK_DISC && isCmd) {
      setPeer(srcCall, srcSSID); storeReversePath(frame, addrCount);
      sendU(U_DM, false, pf);
    }
    return;
  }
  if (!samePeer(srcCall, srcSSID)) return;            // frame from a third party

  // ---- S frames ----
  if ((control & 0x03) == 0x01) {
    uint8_t nr = control >> 5;
    uint8_t s  = control & 0x0F;
    ackUpTo(nr);
    if      (s == S_RR)  { peerBusy = false; }
    else if (s == S_RNR) { peerBusy = true;  }
    else if (s == S_REJ) { peerBusy = false; retransmitFrom(nr); }
    if (isCmd && pf) sendS(S_RR, false, true);        // answer poll
    pumpTx();
    if (v_a == v_s) t1Stop();
    t3Start();
    return;
  }

  // ---- I frame ----
  if ((control & 0x01) == 0) {
    uint8_t ns = (control >> 1) & 0x07;
    uint8_t nr = (control >> 5) & 0x07;
    ackUpTo(nr);
    if (ns == v_r) {                                  // in sequence -> deliver
      uint16_t infoOff = ctrlOff + 2;                 // control + PID
      Serial.print("\r\n");
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print("\r\ncmd: ");
      v_r = seqAdd(v_r, 1);
      rejSent = false;
      sendS(S_RR, false, pf);                         // ack (F=1 if it was a poll)
    } else {                                          // out of sequence
      if (!rejSent) { sendS(S_REJ, false, pf); rejSent = true; }
      else if (pf)  { sendS(S_RR,  false, true); }
    }
    pumpTx();
    t3Start();
    return;
  }
}

// Periodic timer service (call every loop).
void ax25Service(void) {
  uint32_t now = millis();
  if (t1On && (uint32_t)(now - t1At) >= frackMs) {
    if (++n2 > retryMax) {
      if (linkState == LINK_RELEASE) { Serial.print("\r\n*** DISCONNECTED\r\n"); connReset(); if (currentMode == CONVERSE) currentMode = COMMAND; Serial.print("cmd: "); }
      else if (linkState == LINK_SETUP) linkFailure("no answer");
      else linkFailure("retry count exceeded");
      return;
    }
    if      (linkState == LINK_SETUP)   { sendU(U_SABM, true, true); t1Start(); }
    else if (linkState == LINK_RELEASE) { sendU(U_DISC, true, true); t1Start(); }
    else if (linkState == LINK_CONN) {
      if (v_a != v_s) retransmitFrom(v_a);            // go-back-N
      else            { sendS(S_RR, true, true); t1Start(); }
    }
  }
  if (t3On && linkState == LINK_CONN && (uint32_t)(now - t3At) >= T3_MS) {
    if (v_a == v_s) { sendS(S_RR, true, true); t1Start(); }   // idle poll
    t3Start();
  }
}

// ---- Command wrappers ----
void cmdConnect(String arg) {
  arg.trim();
  if (arg.length() == 0) { Serial.println("Usage: CONNECT <call[-ssid]>"); return; }
  if (linkState != LINK_DISC) { Serial.println("Busy - DISCONNECT first."); return; }
  char c[7]; uint8_t s; parseCallSSID(arg, c, &s);
  setPeer(c, s);
  connPathCount = txPathCount;                        // connect via the configured path
  for (uint8_t i = 0; i < txPathCount; i++) { strncpy(connPathCall[i], txPathCalls[i], 7); connPathSSID[i] = txPathSSID[i]; }
  v_s = v_r = v_a = 0; n2 = 0; peerBusy = false; rejSent = false; txQHead = txQTail = txQCount = 0;
  linkState = LINK_SETUP;
  sendU(U_SABM, true, true);
  t1Start();
  Serial.print("*** Connecting to "); printPeer(); Serial.println(" ...");
}

void cmdDisconnect(void) {
  if (linkState == LINK_CONN || linkState == LINK_SETUP) {
    sendU(U_DISC, true, true);
    linkState = LINK_RELEASE; n2 = 0; t1Start();
    Serial.println("*** Disconnecting ...");
    if (currentMode == CONVERSE) currentMode = COMMAND;
  } else Serial.println("Not connected.");
}

void cmdMheard(void) {
  if (heardCount == 0) { Serial.println("Nothing heard yet."); return; }
  Serial.println("Stations heard:");
  uint32_t now = millis();
  for (uint8_t i = 0; i < heardCount; i++) {
    Serial.print("  "); Serial.print(heardCall[i]);
    Serial.print("  ("); Serial.print((now - heardAt[i]) / 1000); Serial.println("s ago)");
  }
}

void cmdStatus(void) {
  Serial.print("Link: ");
  if      (linkState == LINK_DISC)    Serial.println("DISCONNECTED");
  else if (linkState == LINK_SETUP)  { Serial.print("CONNECTING to "); printPeer(); Serial.println(); }
  else if (linkState == LINK_RELEASE) Serial.println("DISCONNECTING");
  else {
    Serial.print("CONNECTED to "); printPeer();
    Serial.print("  V(S)="); Serial.print(v_s);
    Serial.print(" V(R)="); Serial.print(v_r);
    Serial.print(" V(A)="); Serial.print(v_a);
    Serial.print(" unacked="); Serial.println(outstanding());
  }
}

// ============================================================
//  GPS  (NMEA 0183 decoder on a separate serial port)
// ============================================================
// Extract comma-separated field `idx` (0 = sentence id) into out (respects
// empty fields; stops at the '*' checksum delimiter).
void nmeaField(const char *s, uint8_t idx, char *out, uint8_t outSize) {
  uint8_t f = 0, oi = 0;
  out[0] = '\0';
  for (const char *p = s; *p && *p != '*'; p++) {
    if (*p == ',') { f++; if (f > idx) return; continue; }
    if (f == idx && oi < outSize - 1) { out[oi++] = *p; out[oi] = '\0'; }
  }
}

// Validate the NMEA "*HH" checksum (XOR of everything between '$' and '*').
bool nmeaChecksumOK(const char *s) {
  if (s[0] != '$') return false;
  uint8_t sum = 0; const char *p = s + 1;
  while (*p && *p != '*') sum ^= (uint8_t)*p++;
  if (*p != '*') return false;
  uint8_t given = (uint8_t)strtol(p + 1, NULL, 16);
  return sum == given;
}

void parseNMEA(const char *s) {
  if (!nmeaChecksumOK(s)) return;
  const char *type = s + 3;               // skip '$' + 2-char talker (GP/GN/GL...)
  char f[16];

  if (strncmp(type, "RMC", 3) == 0) {     // Recommended Minimum -> position + validity
    nmeaField(s, 2, f, sizeof(f));        // status A=valid V=void
    bool valid = (f[0] == 'A');
    if (valid) {
      nmeaField(s, 1, gpsUtc, sizeof(gpsUtc));
      nmeaField(s, 3, gpsLatRaw, sizeof(gpsLatRaw));
      nmeaField(s, 4, f, sizeof(f)); gpsLatHemi = f[0] ? f[0] : 'N';
      nmeaField(s, 5, gpsLonRaw, sizeof(gpsLonRaw));
      nmeaField(s, 6, f, sizeof(f)); gpsLonHemi = f[0] ? f[0] : 'W';
      gpsLastFixMs = millis();
    }
    gpsFixValid = valid;
  } else if (strncmp(type, "GGA", 3) == 0) {  // fix quality, satellites, altitude
    nmeaField(s, 6, f, sizeof(f));            // fix quality (0 = no fix)
    if (f[0] && f[0] != '0') {
      nmeaField(s, 7, f, sizeof(f)); gpsSats = (uint8_t)atoi(f);
      nmeaField(s, 9, gpsAlt, sizeof(gpsAlt));
    } else {
      gpsSats = 0;
    }
  }
}

void gpsBegin(void) { gpsSerial.begin(gpsBaud); nmeaLen = 0; }
void gpsEnd(void)   { gpsSerial.end(); gpsFixValid = false; }

// Feed available GPS bytes into the line buffer (call every loop).
void gpsService(void) {
  if (!gpsEnabled) return;
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    if (c == '\r') continue;
    if (c == '\n') { if (nmeaLen > 0) { nmea[nmeaLen] = '\0'; parseNMEA(nmea); } nmeaLen = 0; }
    else if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
    else nmeaLen = 0;                     // overflow -> resync on next line
  }
}

void cmdGpsStatus(void) {
  Serial.print("GPS: "); Serial.print(gpsEnabled ? "ON" : "OFF");
  Serial.print("   baud "); Serial.println(gpsBaud);
  if (!gpsEnabled) return;
  Serial.print("Fix:  "); Serial.println(gpsFixValid ? "VALID" : "no fix");
  Serial.print("Sats: "); Serial.println(gpsSats);
  if (gpsFixValid) {
    Serial.print("Pos:  "); Serial.print(gpsLatRaw); Serial.print(gpsLatHemi);
    Serial.print("  ");     Serial.print(gpsLonRaw); Serial.println(gpsLonHemi);
    if (gpsAlt[0]) { Serial.print("Alt:  "); Serial.print(gpsAlt); Serial.println(" m"); }
    if (gpsUtc[0]) { Serial.print("UTC:  "); Serial.println(gpsUtc); }
    Serial.print("Age:  "); Serial.print((millis() - gpsLastFixMs) / 1000); Serial.println(" s");
    char lat[12], lon[13];
    aprsCoord(gpsLatRaw, gpsLatHemi, lat);
    aprsCoord(gpsLonRaw, gpsLonHemi, lon);
    Serial.print("APRS: !"); Serial.print(lat); Serial.print(aprsSymTable);
    Serial.print(lon); Serial.println(aprsSymCode);
  }
}

// ============================================================
//  RX: HDLC parser (called once per decoded data bit)
// ============================================================
static inline void hdlcParse(uint8_t bit) {
  hdlcBits = (hdlcBits << 1) | (bit & 1);

  if (hdlcBits == 0x7E) {                  // ---- FLAG (frame boundary) ----
    if (rxReceiving && rxFrameLen >= 17 && !rxFrameReady) {
      memcpy(rxFrame, rxFrameBuf, rxFrameLen);   // hand raw frame to main loop
      rxRawLen = rxFrameLen;
      rxFrameReady = true;                       // FCS is verified in loop()
    }
    rxReceiving = true;                    // this flag also opens the next frame
    rxFrameLen  = 0;
    rxByte      = 0;
    rxBitCount  = 0;
    return;
  }

  if ((hdlcBits & 0x7F) == 0x7F) {         // ---- 7+ ones: abort / idle ----
    rxReceiving = false;
    return;
  }

  if (!rxReceiving) return;

  if ((hdlcBits & 0x3F) == 0x3E) return;   // stuffed 0 after five 1s -> discard

  rxByte >>= 1;                            // AX.25 is LSB-first
  if (hdlcBits & 0x01) rxByte |= 0x80;
  if (++rxBitCount >= 8) {
    if (rxFrameLen < MAX_FRAME) rxFrameBuf[rxFrameLen++] = rxByte;
    else rxReceiving = false;             // overflow -> drop
    rxBitCount = 0;
    rxByte = 0;
  }
}

// ============================================================
//  Sample-rate ISR (SAMPLERATE Hz): TX DDS or RX demod
// ============================================================
void sampleISR(void) {
  if (transmitting) {
    // ---- Transmit: clock the DDS one sample ----
    if (txSampleInBit == 0) {
      if (txBitIndex >= txBitCount) { txDone = true; return; }
      uint8_t bit = (txBitBuf[txBitIndex >> 3] >> (txBitIndex & 7)) & 1;
      txBitIndex++;
      if (bit == 0) txMark = !txMark;                 // NRZI: 0 = tone change
      txPhaseInc = txMark ? MARK_INC : SPACE_INC;
    }
    txPhase += txPhaseInc;                            // uint8 wraps mod SIN_LEN
    PwmTimer->setCaptureCompare(2, sineTable[txPhase], TICK_COMPARE_FORMAT);
    if (++txSampleInBit >= SAMPLESPERBIT) txSampleInBit = 0;
    return;
  }

  // ---- Receive: read audio and demodulate ----
  // Portable read. For lower jitter, configure ADC1 once for a fixed short
  // sample time and replace this with a direct  (int16_t)ADC1->DR  read.
  int16_t raw = analogRead(RX_PIN);        // 0..4095 (12-bit)
  int8_t  s   = (int8_t)((raw >> 4) - 128);// center to signed 8-bit

  // Delay-line multiply frequency discriminator (delay = SAMPLESPERBIT/2)
  int8_t delayed = delayLine[delayPos];
  iirX[0] = iirX[1];
  iirX[1] = ((int16_t)delayed * s) >> 2;
  iirY[0] = iirY[1];
  iirY[1] = iirX[0] + iirX[1] + (iirY[0] >> 1);   // 1-pole low-pass
  delayLine[delayPos] = s;
  delayPos = (delayPos + 1) & ((SAMPLESPERBIT / 2) - 1);

  // Raw tone decision (sign of the discriminator output)
  sampledBits = (sampledBits << 1) | ((iirY[1] > 0) ? 1 : 0);

  // Digital PLL: nudge phase toward the center of the bit on each edge
  if ((sampledBits ^ (sampledBits >> 1)) & 0x01) {
    if (currentPhase < PHASE_THRES) currentPhase += PHASE_INC;
    else                            currentPhase -= PHASE_INC;
  }
  currentPhase += PHASE_BITS;

  if (currentPhase >= PHASE_MAX) {         // ---- once per bit ----
    currentPhase -= PHASE_MAX;
    actualBits <<= 1;
    uint8_t b3 = sampledBits & 0x07;       // majority vote of last 3 samples
    if (b3 == 0x07 || b3 == 0x06 || b3 == 0x05 || b3 == 0x03) actualBits |= 1;
    // NRZI decode: transition -> 0, no transition -> 1
    uint8_t dataBit = ((actualBits ^ (actualBits >> 1)) & 0x01) ? 0 : 1;
    hdlcParse(dataBit);
  }
}

// ============================================================
//  Deliver a validated frame to the host (KISS or terminal)
// ============================================================
void onRFDataReceived(uint8_t *frame, uint16_t len) {
  if (currentMode == KISS) {
    Serial.write(FEND);
    Serial.write((uint8_t)0x00);           // KISS data-frame command
    for (uint16_t i = 0; i < len; i++) {
      uint8_t c = frame[i];
      if (c == FEND)      { Serial.write(FESC); Serial.write(TFEND); }
      else if (c == FESC) { Serial.write(FESC); Serial.write(TFESC); }
      else                { Serial.write(c); }
    }
    Serial.write(FEND);
  } else {
    bool foundText = false;
    Serial.print("\r\n<RX> ");
    for (uint16_t i = 0; i + 1 < len; i++) {
      if (frame[i] == 0x03 && frame[i + 1] == 0xF0) {   // UI control + PID
        for (uint16_t j = i + 2; j < len; j++) Serial.print((char)frame[j]);
        foundText = true;
        break;
      }
    }
    if (!foundText) Serial.print("[Raw packet - no text payload]");
    if (currentMode == COMMAND) Serial.print("\r\ncmd: ");
    else                        Serial.print("\r\n");
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  ledWrite(true);              // steady ON = ready / OK

  // Build the DDS sine table (centered on mid-scale)
  for (int i = 0; i < SIN_LEN; i++)
    sineTable[i] = (uint8_t)(128.0 + TX_AMPLITUDE * sin((2.0 * PI * i) / SIN_LEN));

  // ADC
  analogReadResolution(12);
  pinMode(RX_PIN, INPUT_ANALOG);
  randomSeed(micros());   // for the digipeater collision-avoidance slot

  // PWM carrier on PA1 (TIM2 CH2)
  PwmTimer = new HardwareTimer(TIM2);
  PwmTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, TX_PIN);
  PwmTimer->setPrescaleFactor(1);
  PwmTimer->setOverflow(PWM_TOP + 1, TICK_FORMAT);   // ARR = 255
  PwmTimer->setCaptureCompare(2, PWM_TOP / 2, TICK_COMPARE_FORMAT);
  PwmTimer->resume();

  // Sample-rate interrupt on TIM3
  SampleTimer = new HardwareTimer(TIM3);
  SampleTimer->setOverflow(SAMPLERATE, HERTZ_FORMAT);
  SampleTimer->attachInterrupt(sampleISR);
  SampleTimer->resume();

  while (!Serial && millis() < 3000);   // native-USB CDC: wait briefly

  Serial.println("\r\n*** STM32 Blue Pill TNC (native AFSK1200) ***");
  Serial.println("Type HELP for commands.");
  Serial.print("cmd: ");
}

// ============================================================
//  Main loop (state machine)
// ============================================================
void loop() {
  updateLED();
  ax25Service();
  gpsService();

  // Verified frame from the modem -> host (KISS) or link layer (terminal)
  if (rxFrameReady) {
    uint16_t len = rxRawLen;
    if (checkFCS(rxFrame, len)) {
      noteActivity();
      recordHeard(rxFrame);
      if (currentMode == KISS) {
        onRFDataReceived(rxFrame, len - 2);          // raw frame to host
      } else {
        ax25HandleRx(rxFrame, len - 2);              // connected mode + monitor
        if (digiEnabled) tryDigipeat(rxFrame, len - 2);
      }
    }
    rxFrameReady = false;
  }

  // Periodic beacon (skipped in KISS mode -- the host handles beaconing there)
  if (beaconEnabled && currentMode != KISS &&
      (uint32_t)(millis() - lastBeaconMs) >= beaconInterval) {
    lastBeaconMs = millis();
    sendBeacon();
  }

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    noteActivity();                         // serial traffic

    // ================= KISS MODE =================
    if (currentMode == KISS) {
      if (b == FEND) {
        if (inKissFrame && kissBufferIndex > 1) {
          uint8_t command = kissBuffer[0] & 0x0F;
          if (command == 0x00) {                 // data frame -> transmit
            sendAX25Frame(kissBuffer + 1, kissBufferIndex - 1);
          } else if (command == 0x0F) {          // KISS "Return" -> exit
            currentMode = COMMAND;
            linkConnected = false;               // LED: leave connected state
            Serial.println("\r\nExited KISS Mode.");
            Serial.print("cmd: ");
          }
        }
        inKissFrame = true;
        kissBufferIndex = 0;
        escapeNext = false;
        continue;
      }

      if (inKissFrame) {
        if (b == FESC) {
          escapeNext = true;
        } else {
          if (escapeNext) {
            if (b == TFEND) b = FEND;
            else if (b == TFESC) b = FESC;
            escapeNext = false;
          }
          if (kissBufferIndex < sizeof(kissBuffer)) kissBuffer[kissBufferIndex++] = b;
          else inKissFrame = false;              // overflow -> drop frame
        }
      }
      continue;
    }

    // ============= COMMAND / CONVERSE MODE =============
    if (b == 0x03 && currentMode == CONVERSE) {  // Ctrl+C -> command mode
      currentMode = COMMAND;
      inputLen = 0;
      Serial.print("\r\ncmd: ");
      continue;
    }

    if (b != '\r' && b != '\n' && b != 0x03) Serial.print((char)b);  // echo

    if (b == '\n' || b == '\r') {
      if (inputLen == 0) {
        if (currentMode == COMMAND) Serial.print("\r\ncmd: ");
        continue;
      }
      inputBuffer[inputLen] = '\0';

      if (currentMode == CONVERSE) {
        Serial.println();
        if (linkState == LINK_CONN) {
          if (inputLen < sizeof(inputBuffer) - 1) inputBuffer[inputLen++] = '\r'; // send CR
          uint16_t off = 0;
          while (off < inputLen) {
            uint16_t chunk = inputLen - off;
            if (chunk > paclen) chunk = paclen;
            if (!enqueueData((uint8_t *)inputBuffer + off, (uint8_t)chunk)) {
              Serial.println("[link busy - not sent]");
              break;
            }
            off += chunk;
          }
          pumpTx();
        } else {
          transmitTextPacket(inputBuffer, inputLen);   // unproto UI
        }
      } else {  // COMMAND
        String raw = String(inputBuffer);       // original case (for BTEXT)
        raw.trim();
        String cmd = raw;
        cmd.toUpperCase();
        Serial.println();

        if (cmd.startsWith("MYCALL ")) {
          String call = cmd.substring(7); call.trim();
          call.toCharArray(myCall, 7);
          Serial.print("MYCALL updated to: "); Serial.println(myCall);
        } else if (cmd.startsWith("MYSSID ")) {
          mySSID = (uint8_t) cmd.substring(7).toInt() & 0x0F;
          Serial.print("MYSSID updated to: "); Serial.println(mySSID);
        } else if (cmd.startsWith("BTEXT ")) {
          String t = raw.substring(6); t.trim();
          t.toCharArray(beaconText, sizeof(beaconText));
          Serial.print("BTEXT set: "); Serial.println(beaconText);
        } else if (cmd.startsWith("BEACON EVERY ")) {
          uint32_t s = (uint32_t) cmd.substring(13).toInt();
          if (s == 0) { beaconEnabled = false; Serial.println("Auto-beacon off."); }
          else {
            beaconInterval = s * 1000UL;
            beaconEnabled  = true;
            lastBeaconMs   = millis();
            Serial.print("Auto-beacon every "); Serial.print(s); Serial.println("s.");
          }
        } else if (cmd == "BEACON OFF") {
          beaconEnabled = false;
          Serial.println("Auto-beacon off.");
        } else if (cmd == "BEACON") {
          sendBeacon();
        } else if (cmd == "DIGI ON") {
          digiEnabled = true;  Serial.println("Digipeater ON.");
        } else if (cmd == "DIGI OFF") {
          digiEnabled = false; Serial.println("Digipeater OFF.");
        } else if (cmd == "DIGI FILL") {
          digiMode = DIGI_FILL; Serial.println("Digi mode: FILL (WIDE1-1 only).");
        } else if (cmd == "DIGI WIDE") {
          digiMode = DIGI_WIDE;
          Serial.print("Digi mode: WIDE (up to WIDEn-"); Serial.print(wideMax);
          Serial.println(").");
        } else if (cmd.startsWith("WIDEMAX ")) {
          int v = cmd.substring(8).toInt();
          if (v < 1) v = 1;
          if (v > 7) v = 7;
          wideMax = (uint8_t)v;
          Serial.print("WIDEMAX = "); Serial.println(wideMax);
        } else if (cmd.startsWith("MYALIAS ")) {
          String a = cmd.substring(8); a.trim();
          if (a == "OFF") { aliasEnabled = false; Serial.println("Digi alias disabled."); }
          else {
            parseCallSSID(a, myAlias, &myAliasSSID);
            aliasEnabled = true;
            Serial.print("Digi alias: "); Serial.print(myAlias);
            Serial.print("-"); Serial.println(myAliasSSID);
          }
        } else if (cmd == "PATH" || cmd == "PATH OFF") {
          txPathCount = 0;
          Serial.println("TX path cleared.");
        } else if (cmd.startsWith("PATH ")) {
          setTxPath(cmd.substring(5));
        } else if (cmd.startsWith("CONNECT ")) {
          cmdConnect(cmd.substring(8));
        } else if (cmd.startsWith("C ")) {
          cmdConnect(cmd.substring(2));
        } else if (cmd == "DISCONNECT" || cmd == "D" || cmd == "BYE") {
          cmdDisconnect();
        } else if (cmd == "MHEARD" || cmd == "MH") {
          cmdMheard();
        } else if (cmd == "MONITOR ON") {
          monitorOn = true;  Serial.println("Monitor ON.");
        } else if (cmd == "MONITOR OFF") {
          monitorOn = false; Serial.println("Monitor OFF.");
        } else if (cmd == "STATUS" || cmd == "CS") {
          cmdStatus();
        } else if (cmd == "FRACK") {
          Serial.print("FRACK "); Serial.print(frackMs / 1000); Serial.println(" s");
        } else if (cmd.startsWith("FRACK ")) {
          long v = cmd.substring(6).toInt();
          if (v < 1) v = 1;
          if (v > 60) v = 60;
          frackMs = (uint32_t)v * 1000UL;
          Serial.print("FRACK = "); Serial.print(v); Serial.println(" s");
        } else if (cmd == "RETRY") {
          Serial.print("RETRY "); Serial.println(retryMax);
        } else if (cmd.startsWith("RETRY ")) {
          long v = cmd.substring(6).toInt();
          if (v < 1) v = 1;
          if (v > 30) v = 30;
          retryMax = (uint8_t)v;
          Serial.print("RETRY = "); Serial.println(retryMax);
        } else if (cmd == "PACLEN") {
          Serial.print("PACLEN "); Serial.println(paclen);
        } else if (cmd.startsWith("PACLEN ")) {
          long v = cmd.substring(7).toInt();
          if (v < 1) v = 1;
          if (v > PACLEN_MAX) v = PACLEN_MAX;
          paclen = (uint8_t)v;
          Serial.print("PACLEN = "); Serial.println(paclen);
        } else if (cmd == "GPS ON") {
          gpsEnabled = true; gpsBegin(); Serial.println("GPS ON.");
        } else if (cmd == "GPS OFF") {
          gpsEnabled = false; gpsEnd(); Serial.println("GPS OFF.");
        } else if (cmd.startsWith("GPS BAUD ")) {
          long b = cmd.substring(9).toInt();
          if (b >= 1200 && b <= 115200) {
            gpsBaud = (uint32_t)b;
            if (gpsEnabled) { gpsEnd(); gpsBegin(); }
            Serial.print("GPS baud "); Serial.println(gpsBaud);
          } else Serial.println("Baud out of range (1200-115200).");
        } else if (cmd == "GPS" || cmd == "GPS STATUS") {
          cmdGpsStatus();
        } else if (cmd == "SYMBOL") {
          Serial.print("Symbol: "); Serial.print(aprsSymTable); Serial.println(aprsSymCode);
        } else if (cmd.startsWith("SYMBOL ")) {
          String s = raw.substring(7); s.trim();      // original case (codes are case-sensitive)
          if (s.length() >= 2) {
            aprsSymTable = s.charAt(0);
            aprsSymCode  = s.charAt(1);
            Serial.print("Symbol set: "); Serial.print(aprsSymTable); Serial.println(aprsSymCode);
          } else {
            Serial.println("Usage: SYMBOL <table><code>   e.g. SYMBOL /-  (house), /> (car)");
          }
        } else if (cmd == "CONV") {
          currentMode = CONVERSE;
          Serial.println("Entering Converse Mode (Ctrl+C to exit)");
        } else if (cmd == "KISS ON") {
          currentMode = KISS;
          linkConnected = true;                  // LED: enter connected state
          inKissFrame = false; kissBufferIndex = 0;
          Serial.println("Entering KISS Mode. Awaiting FEND frames.");
        } else if (cmd == "HELP") {
          Serial.println("Commands:");
          Serial.println("  MYCALL <call>      - Set source callsign");
          Serial.println("  MYSSID <0-15>      - Set source SSID");
          Serial.println("  PATH <a,b,..>|OFF  - TX digi path (e.g. WIDE1-1,WIDE2-1)");
          Serial.println("  CONNECT <call>     - Connect to a station (alias: C)");
          Serial.println("  DISCONNECT         - Close the link (alias: D, BYE)");
          Serial.println("  STATUS             - Show link state (alias: CS)");
          Serial.println("  FRACK [sec]        - Retransmit timeout (T1)");
          Serial.println("  RETRY [n]          - Max retries (N2) before link fails");
          Serial.println("  PACLEN [n]         - Max info bytes per I-frame (1-255)");
          Serial.println("  GPS ON | OFF       - Enable/disable GPS position source");
          Serial.println("  GPS BAUD <n>       - GPS serial speed (default 9600)");
          Serial.println("  GPS                - Show GPS status and current location");
          Serial.println("  SYMBOL <tbl><code> - APRS symbol, e.g. /- house, /> car");
          Serial.println("  MHEARD             - List stations heard (alias: MH)");
          Serial.println("  MONITOR ON | OFF   - Show/hide heard UI frames");
          Serial.println("  BTEXT <text>       - Set beacon text");
          Serial.println("  BEACON             - Send beacon once now");
          Serial.println("  BEACON EVERY <sec> - Auto-beacon interval (0 = off)");
          Serial.println("  BEACON OFF         - Stop auto-beacon");
          Serial.println("  DIGI ON | OFF      - Enable/disable digipeater");
          Serial.println("  DIGI FILL | WIDE   - Fill-in (WIDE1-1) or full WIDEn-N");
          Serial.println("  WIDEMAX <1-7>      - Max N handled in WIDE mode");
          Serial.println("  MYALIAS <call>|OFF - Extra exact-match alias (optional)");
          Serial.println("  CONV               - Converse mode (unproto or connected)");
          Serial.println("  KISS ON            - KISS TNC mode for PC software");
        } else {
          Serial.println("ERROR: Unknown command.");
        }

        if (currentMode == COMMAND) Serial.print("cmd: ");
      }
      inputLen = 0;
    } else {
      if (inputLen < sizeof(inputBuffer) - 1) inputBuffer[inputLen++] = b;
    }
  }
}
