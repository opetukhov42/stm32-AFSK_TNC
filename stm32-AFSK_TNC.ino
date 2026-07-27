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
 *     USB support: "CDC (generic Serial supersede U(S)ART)"
 *     U(S)ART support: "Enabled (generic 'Serial')"
 *
 * HARDWARE (STM32F103C8T6)
 *   RX Audio (AFSK) -> PA0 (ADC_IN0). AC-couple the radio's speaker/discriminator audio.
 *   TX Audio (AFSK) -> PA1 (TIM2_CH2). RC low-pass to the radio mic.
 *   PTT Control     -> PA4. Drives an NPN transistor base to pull radio PTT to GND.
 *   Status LED      -> PC13. Onboard LED (active LOW).
 *   GPS RX (Data)   -> PB11 (USART3_RX). Connect to the GPS module's TX pin.
 *   GPS TX (Data)   -> PB10 (USART3_TX). Connect to the GPS module's RX pin (optional).
 *   Weather RX      -> PA3 (USART2_RX). Connect to the Weather Station's TX pin.
 *   Weather TX      -> PA2 (USART2_TX). Connect to the Weather Station's RX pin (optional).
 */

#include <Arduino.h>
#include <HardwareTimer.h>
#include <math.h>

// ---------------- GPS serial port ----------------
#define GPS_USE_SOFTWARE_SERIAL 0
#define GPS_RX_PIN PB11   // GPS TX -> here (USART3 RX)
#define GPS_TX_PIN PB10   // GPS RX <- here (USART3 TX)
#if GPS_USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
#else
  // Explicitly instantiate Serial3 to satisfy the core's linker requirements
  Uart Serial3(GPS_RX_PIN, GPS_TX_PIN);
  #define gpsSerial Serial3
#endif

// ---------------- Weather Station serial port ----
// Explicitly instantiate Serial2 on PA3 (RX) and PA2 (TX)
Uart Serial2(PA3, PA2);

enum WxMode { WX_OFF, WX_MANUAL, WX_SERIAL2 };
WxMode wxMode = WX_OFF;
char wxData[80] = "000/000g000t000r000p000P000h00b00000"; // Dummy template
char wxSerialBuf[80];
uint8_t wxSerialLen = 0;

// APRS symbol for GPS position beacons (runtime-settable via the SYMBOL command).
char aprsSymTable = '/';
char aprsSymCode  = '>';

// ---------------- Pin map ----------------
#define PTT_PIN PA4   // Moved to PA4 to free up PA2 for USART2 TX
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
//  CRC-16 / X.25 (AX.25 FCS)
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
  crc ^= 0xFFFF;
  return ((uint8_t)(crc & 0xFF) == buf[len - 2]) &&
         ((uint8_t)(crc >> 8)  == buf[len - 1]);
}

// ============================================================
//  TX: build the HDLC bitstream (flags + stuffed data + FCS)
// ============================================================
static inline void txPushBitRaw(uint8_t bit) {
  if ((txBitCount >> 3) >= TX_BITBUF_BYTES) return;
  uint32_t idx = txBitCount >> 3;
  uint8_t  msk = 1 << (txBitCount & 7);
  if (bit) txBitBuf[idx] |=  msk;
  else     txBitBuf[idx] &= ~msk;
  txBitCount++;
}

static inline void txPushDataBit(uint8_t bit) {
  txPushBitRaw(bit);
  if (bit) {
    if (++txStuffOnes == 5) { txPushBitRaw(0); txStuffOnes = 0; }
  } else {
    txStuffOnes = 0;
  }
}

static void txPushByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) txPushDataBit((b >> i) & 1);
}

static void txPushFlag(void) {
  txStuffOnes = 0;
  txPushBitRaw(0);
  for (uint8_t i = 0; i < 6; i++) txPushBitRaw(1);
  txPushBitRaw(0);
}

void triggerPTT(bool state) {
  digitalWrite(PTT_PIN, state ? HIGH : LOW);
  if (state) delay(250);
  else       delay(50);
}

void sendAX25Frame(uint8_t *data, uint16_t len) {
  if (len == 0 || len > (MAX_FRAME - 2)) return;
  noteActivity();

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) crc = crcUpdate(crc, data[i]);
  crc ^= 0xFFFF;
  uint8_t fcsLo = crc & 0xFF;
  uint8_t fcsHi = (crc >> 8) & 0xFF;

  txBitCount = 0;
  txStuffOnes = 0;
  for (uint16_t i = 0; i < TX_PREAMBLE_FLAGS; i++) txPushFlag();
  for (uint16_t i = 0; i < len; i++)               txPushByte(data[i]);
  txPushByte(fcsLo);
  txPushByte(fcsHi);
  for (uint16_t i = 0; i < TX_TAIL_FLAGS; i++)     txPushFlag();

  txBitIndex = 0;
  txSampleInBit = 0;
  txPhase = 0;
  txMark = true;
  txPhaseInc = MARK_INC;
  txDone = false;

  triggerPTT(true);
  transmitting = true;
  while (!txDone) { /* wait */ }
  transmitting = false;

  PwmTimer->setCaptureCompare(2, PWM_TOP / 2, TICK_COMPARE_FORMAT);
  triggerPTT(false);
}

void encodeAddress(const char *call, uint8_t ssid, uint8_t *out, bool isLast) {
  uint8_t n = strlen(call);
  for (uint8_t i = 0; i < 6; i++) out[i] = ((i < n) ? call[i] : ' ') << 1;
  out[6] = ((ssid & 0x0F) << 1) | 0x60;
  if (isLast) out[6] |= 0x01;
}

void buildAndSendUI(const char *payload, uint16_t len) {
  uint8_t frame[MAX_FRAME];
  uint16_t p = 0;
  encodeAddress(destCall, destSSID, &frame[p], false);            p += 7;
  encodeAddress(myCall,   mySSID,   &frame[p], txPathCount == 0); p += 7;
  for (uint8_t i = 0; i < txPathCount; i++) {
    encodeAddress(txPathCalls[i], txPathSSID[i], &frame[p], i == txPathCount - 1);
    p += 7;
  }
  frame[p++] = 0x03;
  frame[p++] = 0xF0;
  for (uint16_t i = 0; i < len && p < MAX_FRAME - 2; i++) frame[p++] = payload[i];
  sendAX25Frame(frame, p);
}

void transmitTextPacket(char *payload, uint8_t len) {
  buildAndSendUI(payload, len);
  Serial.println("\r\n[TX OK]");
}

void aprsCoord(const char *raw, char hemi, char *out) {
  int dot = -1;
  for (int i = 0; raw[i]; i++) if (raw[i] == '.') { dot = i; break; }
  if (dot < 0) { out[0] = '\0'; return; }
  int end = dot + 3;
  int oi = 0;
  for (int k = 0; k < end && raw[k]; k++) out[oi++] = raw[k];
  out[oi++] = hemi;
  out[oi] = '\0';
}

void buildBeaconPayload(char *out, int outSize) {
  if (gpsEnabled && gpsFixValid) {
    char lat[12], lon[13];
    aprsCoord(gpsLatRaw, gpsLatHemi, lat);
    aprsCoord(gpsLonRaw, gpsLonHemi, lon);
    
    if (wxMode != WX_OFF) {
      // APRS WX format uses '_' as the symbol code, followed by the WX data
      snprintf(out, outSize, "!%s%c%s_%s%s", lat, aprsSymTable, lon, wxData, beaconText);
    } else {
      snprintf(out, outSize, "!%s%c%s%c%s", lat, aprsSymTable, lon, aprsSymCode, beaconText);
    }
  } else {
    if (wxMode != WX_OFF) {
      // No GPS, but WX active
      snprintf(out, outSize, "%s%s", wxData, beaconText);
    } else {
      strncpy(out, beaconText, outSize - 1);
      out[outSize - 1] = '\0';
    }
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
bool addrMatches(const uint8_t *addr, const char *call, uint8_t ssid) {
  uint8_t n = strlen(call);
  for (uint8_t i = 0; i < 6; i++) {
    char c = (char)((addr[i] >> 1) & 0x7F);
    char e = (i < n) ? call[i] : ' ';
    if (c != e) return false;
  }
  return (((addr[6] >> 1) & 0x0F) == (ssid & 0x0F));
}

bool insertAddress(uint8_t *frame, uint16_t *len, uint16_t off, const char *call, uint8_t ssid, bool hbit) {
  if (*len + 7 > (MAX_FRAME - 2)) return false;
  memmove(frame + off + 7, frame + off, *len - off);
  encodeAddress(call, ssid, &frame[off], false);
  if (hbit) frame[off + 6] |= 0x80;
  *len += 7;
  return true;
}

uint16_t packetSignature(uint8_t *frame, uint16_t len) {
  uint16_t i = 0; bool end = false;
  while (i + 7 <= len) { if (frame[i + 6] & 0x01) { end = true; break; } i += 7; }
  if (!end) return 0;
  uint16_t infoStart = i + 7;
  uint16_t crc = 0xFFFF;
  for (uint8_t k = 0;  k < 6;  k++)  crc = crcUpdate(crc, frame[k]);
  for (uint8_t k = 7;  k < 13; k++)  crc = crcUpdate(crc, frame[k]);
  crc = crcUpdate(crc, (uint8_t)((frame[13] >> 1) & 0x0F));
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

bool isWideN(const uint8_t *addr, uint8_t *n, uint8_t *N) {
  char c[6];
  for (uint8_t k = 0; k < 6; k++) c[k] = (char)((addr[k] >> 1) & 0x7F);
  if (!(c[0] == 'W' && c[1] == 'I' && c[2] == 'D' && c[3] == 'E' && c[4] >= '1' && c[4] <= '7' && c[5] == ' '))
    return false;
  *n = c[4] - '0';
  *N = (addr[6] >> 1) & 0x0F;
  return (*N >= 1 && *N <= *n);
}

void tryDigipeat(uint8_t *frame, uint16_t len) {
  if (!digiEnabled) return;
  int addrCount = 0; uint16_t i = 0; bool foundEnd = false;
  while (i + 7 <= len && addrCount < 2 + 8) {
    addrCount++;
    if (frame[i + 6] & 0x01) { foundEnd = true; break; }
    i += 7;
  }
  if (!foundEnd || addrCount < 3) return;
  if (addrMatches(&frame[7], myCall, mySSID)) return;

  int slot = -1;
  for (int a = 2; a < addrCount; a++)
    if (!(frame[a * 7 + 6] & 0x80)) { slot = a; break; }
  if (slot < 0) return;

  uint16_t off     = (uint16_t)slot * 7;
  bool     wasLast = frame[off + 6] & 0x01;
  uint8_t  digis   = addrCount - 2;
  bool     doTx    = false;

  if (addrMatches(&frame[off], myCall, mySSID)) {
    frame[off + 6] |= 0x80;
    doTx = true;
  } else if (aliasEnabled && addrMatches(&frame[off], myAlias, myAliasSSID)) {
    encodeAddress(myCall, mySSID, &frame[off], wasLast);
    frame[off + 6] |= 0x80;
    doTx = true;
  } else {
    uint8_t n, N;
    if (isWideN(&frame[off], &n, &N)) {
      bool ok = (digiMode == DIGI_FILL) ? (n == 1 && N == 1) : (N <= wideMax);
      if (ok) {
        if (N == 1) {
          encodeAddress(myCall, mySSID, &frame[off], wasLast);
          frame[off + 6] |= 0x80;
        } else {
          frame[off + 6] = (((N - 1) & 0x0F) << 1) | 0x60 | (wasLast ? 0x01 : 0);
          if (digis < 8) insertAddress(frame, &len, off, myCall, mySSID, true);
        }
        doTx = true;
      }
    }
  }
  if (!doTx) return;

  uint16_t sig = packetSignature(frame, len);
  if (sig && seenRecently(sig)) return;
  if (sig) rememberPacket(sig);

  delay(random(60, 260));
  sendAX25Frame(frame, len);
  Serial.println("\r\n[DIGI repeated]");
  if (currentMode == COMMAND) Serial.print("cmd: ");
}

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
//  AX.25 v2.2 Connected Mode
// ============================================================
enum LinkState { LINK_DISC, LINK_SETUP, LINK_CONN, LINK_RELEASE };
LinkState linkState = LINK_DISC;

char    peerCall[7] = "";
uint8_t peerSSID = 0;

char    connPathCall[MAX_PATH][7];
uint8_t connPathSSID[MAX_PATH];
uint8_t connPathCount = 0;

#define AX_MOD     8
#define AX_WINDOW  4
uint8_t v_s = 0, v_r = 0, v_a = 0;
bool    peerBusy = false;
bool    rejSent  = false;

#define T1_MS_DEF   4000UL
#define T3_MS       180000UL
#define N2_DEF      10
uint32_t frackMs  = T1_MS_DEF;
uint8_t  retryMax = N2_DEF;
uint32_t t1At = 0; bool t1On = false;
uint32_t t3At = 0; bool t3On = false;
uint8_t  n2 = 0;

#define PACLEN_MAX 255
#define PACLEN_DEF 128
uint8_t paclen = PACLEN_DEF;
struct Seg { uint8_t data[PACLEN_MAX]; uint8_t len; };
Seg     iStore[AX_MOD];
Seg     txQ[AX_MOD];
uint8_t txQHead = 0, txQTail = 0, txQCount = 0;

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

#define MHEARD_SIZE 8
char     heardCall[MHEARD_SIZE][10];
uint32_t heardAt[MHEARD_SIZE];
uint8_t  heardCount = 0;

static inline uint8_t seqAdd(uint8_t a, uint8_t b) { return (a + b) & (AX_MOD - 1); }
static inline uint8_t outstanding(void)            { return (v_s - v_a) & (AX_MOD - 1); }

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

uint16_t connHeader(uint8_t *f, bool isCommand) {
  encodeAddress(peerCall, peerSSID, &f[0], false);
  encodeAddress(myCall,   mySSID,   &f[7], connPathCount == 0);
  if (isCommand) { f[6] |= 0x80; f[13] &= ~0x80; }
  else           { f[6] &= ~0x80; f[13] |= 0x80; }
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
  uint16_t p = connHeader(f, true);
  f[p++] = (v_r << 5) | (pf ? PF_BIT : 0) | (ns << 1);
  f[p++] = 0xF0;
  for (uint8_t i = 0; i < iStore[ns].len; i++) f[p++] = iStore[ns].data[i];
  sendAX25Frame(f, p);
}

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
  linkConnected = false;
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

  if ((control & ~PF_BIT) == U_UI) {
    if (monitorOn) {
      uint16_t infoOff = ctrlOff + 2;
      Serial.print("\r\n<UI "); Serial.print(srcCall);
      if (srcSSID) { Serial.print("-"); Serial.print(srcSSID); }
      Serial.print("> ");
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print("\r\ncmd: "); else Serial.print("\r\n");
    }
    return;
  }

  if (!destUs) return;
  if (addrMatches(&frame[7], myCall, mySSID)) return;

  if ((control & 0x03) == 0x03) {
    uint8_t u = control & ~PF_BIT;
    if (u == U_SABM) {
      if (linkState == LINK_CONN && !samePeer(srcCall, srcSSID)) return;
      setPeer(srcCall, srcSSID);
      storeReversePath(frame, addrCount);
      v_s = v_r = v_a = 0; n2 = 0; peerBusy = false; rejSent = false;
      txQHead = txQTail = txQCount = 0;
      linkState = LINK_CONN;
      linkConnected = true;
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
        linkConnected = true;
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

  if (linkState != LINK_CONN) {
    if (linkState == LINK_DISC && isCmd) {
      setPeer(srcCall, srcSSID); storeReversePath(frame, addrCount);
      sendU(U_DM, false, pf);
    }
    return;
  }
  if (!samePeer(srcCall, srcSSID)) return;

  if ((control & 0x03) == 0x01) {
    uint8_t nr = control >> 5;
    uint8_t s  = control & 0x0F;
    ackUpTo(nr);
    if      (s == S_RR)  { peerBusy = false; }
    else if (s == S_RNR) { peerBusy = true;  }
    else if (s == S_REJ) { peerBusy = false; retransmitFrom(nr); }
    if (isCmd && pf) sendS(S_RR, false, true);
    pumpTx();
    if (v_a == v_s) t1Stop();
    t3Start();
    return;
  }

  if ((control & 0x01) == 0) {
    uint8_t ns = (control >> 1) & 0x07;
    uint8_t nr = (control >> 5) & 0x07;
    ackUpTo(nr);
    if (ns == v_r) {
      uint16_t infoOff = ctrlOff + 2;
      Serial.print("\r\n");
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print("\r\ncmd: ");
      v_r = seqAdd(v_r, 1);
      rejSent = false;
      sendS(S_RR, false, pf);
    } else {
      if (!rejSent) { sendS(S_REJ, false, pf); rejSent = true; }
      else if (pf)  { sendS(S_RR,  false, true); }
    }
    pumpTx();
    t3Start();
    return;
  }
}

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
      if (v_a != v_s) retransmitFrom(v_a);
      else            { sendS(S_RR, true, true); t1Start(); }
    }
  }
  if (t3On && linkState == LINK_CONN && (uint32_t)(now - t3At) >= T3_MS) {
    if (v_a == v_s) { sendS(S_RR, true, true); t1Start(); }
    t3Start();
  }
}

void cmdConnect(String arg) {
  arg.trim();
  if (arg.length() == 0) { Serial.println("Usage: CONNECT <call[-ssid]>"); return; }
  if (linkState != LINK_DISC) { Serial.println("Busy - DISCONNECT first."); return; }
  char c[7]; uint8_t s; parseCallSSID(arg, c, &s);
  setPeer(c, s);
  connPathCount = txPathCount;
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
//  GPS
// ============================================================
void nmeaField(const char *s, uint8_t idx, char *out, uint8_t outSize) {
  uint8_t f = 0, oi = 0;
  out[0] = '\0';
  for (const char *p = s; *p && *p != '*'; p++) {
    if (*p == ',') { f++; if (f > idx) return; continue; }
    if (f == idx && oi < outSize - 1) { out[oi++] = *p; out[oi] = '\0'; }
  }
}

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
  const char *type = s + 3;
  char f[16];

  if (strncmp(type, "RMC", 3) == 0) {
    nmeaField(s, 2, f, sizeof(f));
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
  } else if (strncmp(type, "GGA", 3) == 0) {
    nmeaField(s, 6, f, sizeof(f));
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

void gpsService(void) {
  if (!gpsEnabled) return;
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    if (c == '\r') continue;
    if (c == '\n') { if (nmeaLen > 0) { nmea[nmeaLen] = '\0'; parseNMEA(nmea); } nmeaLen = 0; }
    else if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
    else nmeaLen = 0;
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
//  Weather Service
// ============================================================
void wxService(void) {
  if (wxMode != WX_SERIAL2) return;
  
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (wxSerialLen > 0) {
        wxSerialBuf[wxSerialLen] = '\0';
        strncpy(wxData, wxSerialBuf, sizeof(wxData) - 1);
        wxSerialLen = 0; // Reset for next line
      }
    } else if (wxSerialLen < sizeof(wxSerialBuf) - 1) {
      wxSerialBuf[wxSerialLen++] = c;
    }
  }
}

// ============================================================
//  RX: HDLC parser
// ============================================================
static inline void hdlcParse(uint8_t bit) {
  hdlcBits = (hdlcBits << 1) | (bit & 1);

  if (hdlcBits == 0x7E) {
    if (rxReceiving && rxFrameLen >= 17 && !rxFrameReady) {
      memcpy(rxFrame, rxFrameBuf, rxFrameLen);
      rxRawLen = rxFrameLen;
      rxFrameReady = true;
    }
    rxReceiving = true;
    rxFrameLen  = 0;
    rxByte      = 0;
    rxBitCount  = 0;
    return;
  }

  if ((hdlcBits & 0x7F) == 0x7F) {
    rxReceiving = false;
    return;
  }

  if (!rxReceiving) return;

  if ((hdlcBits & 0x3F) == 0x3E) return;

  rxByte >>= 1;
  if (hdlcBits & 0x01) rxByte |= 0x80;
  if (++rxBitCount >= 8) {
    if (rxFrameLen < MAX_FRAME) rxFrameBuf[rxFrameLen++] = rxByte;
    else rxReceiving = false;
    rxBitCount = 0;
    rxByte = 0;
  }
}

// ============================================================
//  Sample-rate ISR (SAMPLERATE Hz)
// ============================================================
void sampleISR(void) {
  if (transmitting) {
    if (txSampleInBit == 0) {
      if (txBitIndex >= txBitCount) { txDone = true; return; }
      uint8_t bit = (txBitBuf[txBitIndex >> 3] >> (txBitIndex & 7)) & 1;
      txBitIndex++;
      if (bit == 0) txMark = !txMark;
      txPhaseInc = txMark ? MARK_INC : SPACE_INC;
    }
    txPhase += txPhaseInc;
    PwmTimer->setCaptureCompare(2, sineTable[txPhase], TICK_COMPARE_FORMAT);
    if (++txSampleInBit >= SAMPLESPERBIT) txSampleInBit = 0;
    return;
  }

  int16_t raw = analogRead(RX_PIN);
  int8_t  s   = (int8_t)((raw >> 4) - 128);

  int8_t delayed = delayLine[delayPos];
  iirX[0] = iirX[1];
  iirX[1] = ((int16_t)delayed * s) >> 2;
  iirY[0] = iirY[1];
  iirY[1] = iirX[0] + iirX[1] + (iirY[0] >> 1);
  delayLine[delayPos] = s;
  delayPos = (delayPos + 1) & ((SAMPLESPERBIT / 2) - 1);

  sampledBits = (sampledBits << 1) | ((iirY[1] > 0) ? 1 : 0);

  if ((sampledBits ^ (sampledBits >> 1)) & 0x01) {
    if (currentPhase < PHASE_THRES) currentPhase += PHASE_INC;
    else                            currentPhase -= PHASE_INC;
  }
  currentPhase += PHASE_BITS;

  if (currentPhase >= PHASE_MAX) {
    currentPhase -= PHASE_MAX;
    actualBits <<= 1;
    uint8_t b3 = sampledBits & 0x07;
    if (b3 == 0x07 || b3 == 0x06 || b3 == 0x05 || b3 == 0x03) actualBits |= 1;
    uint8_t dataBit = ((actualBits ^ (actualBits >> 1)) & 0x01) ? 0 : 1;
    hdlcParse(dataBit);
  }
}

void onRFDataReceived(uint8_t *frame, uint16_t len) {
  if (currentMode == KISS) {
    Serial.write(FEND);
    Serial.write((uint8_t)0x00);
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
      if (frame[i] == 0x03 && frame[i + 1] == 0xF0) {
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
  ledWrite(true);

  for (int i = 0; i < SIN_LEN; i++)
    sineTable[i] = (uint8_t)(128.0 + TX_AMPLITUDE * sin((2.0 * PI * i) / SIN_LEN));

  analogReadResolution(12);
  pinMode(RX_PIN, INPUT_ANALOG);
  randomSeed(micros());

  PwmTimer = new HardwareTimer(TIM2);
  PwmTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, TX_PIN);
  PwmTimer->setPrescaleFactor(1);
  PwmTimer->setOverflow(PWM_TOP + 1, TICK_FORMAT);
  PwmTimer->setCaptureCompare(2, PWM_TOP / 2, TICK_COMPARE_FORMAT);
  PwmTimer->resume();

  SampleTimer = new HardwareTimer(TIM3);
  SampleTimer->setOverflow(SAMPLERATE, HERTZ_FORMAT);
  SampleTimer->attachInterrupt(sampleISR);
  SampleTimer->resume();

  while (!Serial && millis() < 3000);

  Serial.println("\r\n*** STM32 Blue Pill TNC (native AFSK1200) ***");
  Serial.println("Type HELP for commands.");
  Serial.print("cmd: ");
}

// ============================================================
//  Main loop
// ============================================================
void loop() {
  updateLED();
  ax25Service();
  gpsService();
  wxService();

  if (rxFrameReady) {
    uint16_t len = rxRawLen;
    if (checkFCS(rxFrame, len)) {
      noteActivity();
      recordHeard(rxFrame);
      if (currentMode == KISS) {
        onRFDataReceived(rxFrame, len - 2);
      } else {
        ax25HandleRx(rxFrame, len - 2);
        if (digiEnabled) tryDigipeat(rxFrame, len - 2);
      }
    }
    rxFrameReady = false;
  }

  if (beaconEnabled && currentMode != KISS &&
      (uint32_t)(millis() - lastBeaconMs) >= beaconInterval) {
    lastBeaconMs = millis();
    sendBeacon();
  }

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    noteActivity();

    // ================= KISS MODE =================
    if (currentMode == KISS) {
      if (b == FEND) {
        if (inKissFrame && kissBufferIndex > 1) {
          uint8_t command = kissBuffer[0] & 0x0F;
          if (command == 0x00) {
            sendAX25Frame(kissBuffer + 1, kissBufferIndex - 1);
          } else if (command == 0x0F) {
            currentMode = COMMAND;
            linkConnected = false;
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
          else inKissFrame = false;
        }
      }
      continue;
    }

    // ============= COMMAND / CONVERSE MODE =============
    if (b == 0x03 && currentMode == CONVERSE) {
      currentMode = COMMAND;
      inputLen = 0;
      Serial.print("\r\ncmd: ");
      continue;
    }

    if (b != '\r' && b != '\n' && b != 0x03) Serial.print((char)b);

    if (b == '\n' || b == '\r') {
      if (inputLen == 0) {
        if (currentMode == COMMAND) Serial.print("\r\ncmd: ");
        continue;
      }
      inputBuffer[inputLen] = '\0';

      if (currentMode == CONVERSE) {
        Serial.println();
        if (linkState == LINK_CONN) {
          if (inputLen < sizeof(inputBuffer) - 1) inputBuffer[inputLen++] = '\r';
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
          transmitTextPacket(inputBuffer, inputLen);
        }
      } else {
        String raw = String(inputBuffer);
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
          String s = raw.substring(7); s.trim();
          if (s.length() >= 2) {
            aprsSymTable = s.charAt(0);
            aprsSymCode  = s.charAt(1);
            Serial.print("Symbol set: "); Serial.print(aprsSymTable); Serial.println(aprsSymCode);
          } else {
            Serial.println("Usage: SYMBOL <table><code>   e.g. SYMBOL /-  (house), /> (car)");
          }
        } else if (cmd == "WX OFF") {
          wxMode = WX_OFF; 
          Serial.println("Weather station disabled.");
        } else if (cmd == "WX MANUAL") {
          wxMode = WX_MANUAL; 
          Serial.println("Weather mode: MANUAL.");
        } else if (cmd == "WX SERIAL2") {
          wxMode = WX_SERIAL2;
          Serial2.begin(9600); // Set to match your hardware station's baud rate
          Serial.println("Weather mode: SERIAL2 (9600 baud).");
        } else if (cmd.startsWith("WX SET ")) {
          String w = raw.substring(7);
          w.trim();
          w.toCharArray(wxData, sizeof(wxData));
          Serial.print("Weather data set to: "); Serial.println(wxData);
        } else if (cmd == "CONV") {
          currentMode = CONVERSE;
          Serial.println("Entering Converse Mode (Ctrl+C to exit)");
        } else if (cmd == "KISS ON") {
          currentMode = KISS;
          linkConnected = true;
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
          Serial.println("  WX OFF             - Disable weather station telemetry");
          Serial.println("  WX MANUAL          - Enable manual weather data entry via WX SET");
          Serial.println("  WX SERIAL2         - Enable weather data parsing from USART2 (PA3)");
          Serial.println("  WX SET <data>      - Set manual telemetry (e.g. 270/010g015t072...)");
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