/*
 * STM32 Blue Pill AFSK 1200 TNC - Dual Mode (KISS + Command)
 * =========================================================
 * Self-contained AX.25 / Bell-202 (1200 baud) modem. No external AFSK
 * library required -- everything (modulation, demodulation, HDLC, CRC)
 * is implemented below. 
 *
 * PERSISTENCE METHOD: Safe Direct Flash Write
 * Completely avoids EEPROM libraries. Safely writes to the final 
 * 1KB Flash page using raw registers and byte-by-byte extraction 
 * to entirely eliminate ARM Misaligned Read HardFaults.
 *
 * TOOLCHAIN
 *   Arduino IDE with the OFFICIAL STMicroelectronics core:
 *     Board:  "Generic STM32F1 series" -> "BluePill F103C8" (64K)
 *     USB support: "CDC (generic Serial supersede U(S)ART)"
 *     U(S)ART support: "Enabled (generic 'Serial')"
 *
 * HARDWARE (STM32F103C8T6)
 *   RX Audio (AFSK) -> PA0 (ADC_IN0). AC-couple the radio's audio.
 *   TX Audio (AFSK) -> PA1 (TIM2_CH2). RC low-pass to the radio mic.
 *   PTT Control     -> PA4. Drives an NPN transistor base to pull radio PTT to GND.
 *   Status LED      -> PC13. Onboard LED (active LOW).
 *   GPS RX (Data)   -> PB11 (USART3_RX). Connect to GPS TX.
 *   Weather RX      -> PA3  (USART2_RX). Connect to WX TX.
 */

#include <Arduino.h>
#include <HardwareTimer.h>
#include <math.h>

// Cross-core compatibility definitions for Flash Registers
#ifndef FLASH_SR_BSY
#define FLASH_SR_BSY (1 << 0)
#endif
#ifndef FLASH_CR_PER
#define FLASH_CR_PER (1 << 1)
#endif
#ifndef FLASH_CR_STRT
#define FLASH_CR_STRT (1 << 6)
#endif
#ifndef FLASH_CR_PG
#define FLASH_CR_PG (1 << 0)
#endif
#ifndef FLASH_CR_LOCK
#define FLASH_CR_LOCK (1 << 7)
#endif

enum GpsMode { GPS_OFF, GPS_SERIAL, GPS_STATIC };
enum WxMode { WX_OFF, WX_MANUAL, WX_SERIAL };
enum DigiMode { DIGI_FILL, DIGI_WIDE };

#define MAX_PATH 2

// ============================================================
//  Safe Direct Flash Storage Structure
// ============================================================
#define FLASH_CONFIG_ADDR  0x0800FC00 // Last 1KB page of 64K Flash
#define FLASH_MAGIC        0xB2A5

#pragma pack(push, 1)
struct ConfigStruct {
  uint16_t magic;
  char     myCall[7];
  uint8_t  mySSID;
  char     beaconText[45];
  bool     beaconEnabled;
  uint32_t beaconInterval;
  bool     digiEnabled;
  uint8_t  digiMode;
  uint8_t  wideMax;
  bool     aliasEnabled;
  char     myAlias[7];
  uint8_t  myAliasSSID;
  char     txPathCalls[MAX_PATH][7];
  uint8_t  txPathSSID[MAX_PATH];
  uint8_t  txPathCount;
  uint32_t frackMs;
  uint8_t  retryMax;
  uint8_t  paclen;
  uint8_t  gpsMode;
  uint32_t gpsBaud;
  char     aprsSymTable;
  char     aprsSymCode;
  uint8_t  wxMode;
  char     wxData[35];
  char     gpsLatRaw[12];
  char     gpsLatHemi;
  char     gpsLonRaw[12];
  char     gpsLonHemi;
};
#pragma pack(pop)

// =========================================================
//  DEFAULT CONFIGURATION (Populated on fresh boot/format)
// =========================================================
#define DEF_MYCALL         "NOCALL"
#define DEF_MYSSID         0
#define DEF_BEACON_TEXT    "STM32 TNC"
#define DEF_BEACON_EN      false
#define DEF_BEACON_SEC     600
#define DEF_DIGI_EN        false
#define DEF_DIGI_MODE      DIGI_FILL // Or DIGI_WIDE
#define DEF_WIDE_MAX       2
#define DEF_ALIAS_EN       false
#define DEF_ALIAS          ""
#define DEF_ALIAS_SSID     0
#define DEF_TX_PATH_CALL1  "WIDE1"
#define DEF_TX_PATH_SSID1  1
#define DEF_TX_PATH_CALL2  "WIDE2"
#define DEF_TX_PATH_SSID2  1
#define DEF_TX_PATH_CNT    0         // Set to 1 or 2 to enable path by default
#define DEF_FRACK_SEC      4
#define DEF_RETRY_MAX      10
#define DEF_PACLEN         128
#define DEF_GPS_MODE       GPS_OFF
#define DEF_GPS_BAUD       9600
#define DEF_GPS_LAT        "0000.00"
#define DEF_GPS_LATH       'N'
#define DEF_GPS_LON        "00000.00"
#define DEF_GPS_LONH       'W'
#define DEF_APRS_SYM_TBL   '/'
#define DEF_APRS_SYM_CODE  '>'
#define DEF_WX_MODE        WX_OFF
#define DEF_WX_DATA        "000/000g000t000r000p000h00b00000"

// ---------------- Serial ports ----------------
#define GPS_USE_SOFTWARE_SERIAL 0
#define GPS_RX_PIN PB11   
#define GPS_TX_PIN PB10   
#if GPS_USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
#else
  Uart Serial3(GPS_RX_PIN, GPS_TX_PIN);
  #define gpsSerial Serial3
#endif

#define WX_USE_SOFTWARE_SERIAL 0
#define WX_RX_PIN PA3     
#define WX_TX_PIN PA2     
#if WX_USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial wxSerial(WX_RX_PIN, WX_TX_PIN);
#else
  Uart Serial2(WX_RX_PIN, WX_TX_PIN);
  #define wxSerial Serial2
#endif

WxMode wxMode = WX_OFF;
char wxData[35] = "";
char wxSerialBuf[35];
uint8_t wxSerialLen = 0;

char aprsSymTable = '/';
char aprsSymCode  = '>';

// ---------------- Pin map ----------------
#define PTT_PIN PA4
#define TX_PIN  PA1   
#define RX_PIN  PA0   
#define LED_PIN         PC13
#define LED_ACTIVE_LOW  1

// ---------------- Modem constants ----------------
#define SAMPLERATE      9600
#define BITRATE         1200
#define SAMPLESPERBIT   (SAMPLERATE / BITRATE)
#define MARK_FREQ       1200
#define SPACE_FREQ      2200

#define SIN_LEN         256
#define PWM_TOP         255
#define TX_AMPLITUDE    110
static uint8_t sineTable[SIN_LEN];
#define MARK_INC        ((uint8_t)((uint32_t)SIN_LEN * MARK_FREQ  / SAMPLERATE))
#define SPACE_INC       ((uint8_t)((uint32_t)SIN_LEN * SPACE_FREQ / SAMPLERATE))

#define PHASE_BITS      8
#define PHASE_INC       3
#define PHASE_MAX       (SAMPLESPERBIT * PHASE_BITS)
#define PHASE_THRES     (PHASE_MAX / 2)

#define TX_PREAMBLE_FLAGS  32
#define TX_TAIL_FLAGS      3

#define MAX_FRAME       340
#define TX_BITBUF_BYTES  700

HardwareTimer *PwmTimer;
HardwareTimer *SampleTimer;

volatile bool transmitting = false;
volatile bool txDone       = false;

static uint8_t  txBitBuf[TX_BITBUF_BYTES];
static uint32_t txBitCount = 0;
static uint8_t  txStuffOnes = 0;
volatile uint32_t txBitIndex = 0;
static uint8_t  txSampleInBit = 0;
static uint8_t  txPhase = 0;
static uint8_t  txPhaseInc = MARK_INC;
static bool     txMark = true;

static int16_t iirX[2], iirY[2];
static int8_t  delayLine[SAMPLESPERBIT / 2];
static uint8_t delayPos = 0;
static uint8_t sampledBits = 0;
static int8_t  currentPhase = 0;
static uint8_t actualBits = 0;

static uint8_t  hdlcBits = 0;
static uint8_t  rxByte = 0;
static uint8_t  rxBitCount = 0;
static bool     rxReceiving = false;
static uint8_t  rxFrameBuf[MAX_FRAME];
static uint16_t rxFrameLen = 0;

static uint8_t           rxFrame[MAX_FRAME];
volatile uint16_t        rxRawLen = 0;
volatile bool            rxFrameReady = false;

enum TNCMode { COMMAND, CONVERSE, KISS };
TNCMode currentMode = COMMAND;

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

char    myCall[7]   = "NOCALL";
uint8_t mySSID      = 0;
char    destCall[7] = "APRS  ";
uint8_t destSSID    = 0;

char     beaconText[45]  = "";
bool     beaconEnabled   = false;
uint32_t beaconInterval  = 600000UL;
uint32_t lastBeaconMs    = 0;

GpsMode  gpsMode      = GPS_OFF;
uint32_t gpsBaud      = 9600;
bool     gpsFixValid  = false;
char     gpsLatRaw[12] = "";
char     gpsLonRaw[12] = "";
char     gpsLatHemi   = 'N';
char     gpsLonHemi   = 'W';
uint8_t  gpsSats      = 0;
char     gpsAlt[10]   = "";
char     gpsUtc[12]   = "";
uint32_t gpsLastFixMs = 0;
char     nmea[100];
uint8_t  nmeaLen = 0;

bool     digiEnabled  = false;
DigiMode digiMode     = DIGI_FILL;
uint8_t  wideMax      = 2;
bool     aliasEnabled = false;
char     myAlias[7]   = "";
uint8_t  myAliasSSID  = 0;

#define DEDUPE_SIZE  16
#define DEDUPE_MS    30000UL
uint16_t dupeSig[DEDUPE_SIZE];
uint32_t dupeAt[DEDUPE_SIZE];
uint8_t  dupeIdx = 0;

uint32_t lastActivityMs = 0;
bool     linkConnected  = false;
uint32_t ledLastToggle  = 0;
bool     ledState       = true;

char    txPathCalls[MAX_PATH][7];
uint8_t txPathSSID[MAX_PATH];
uint8_t txPathCount = 0;

void ledWrite(bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(LED_PIN, on ? LOW : HIGH);
#else
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}

void noteActivity(void) { lastActivityMs = millis(); }

void updateLED(void) {
  uint32_t now = millis();
  uint16_t period;
  if (linkConnected)                                            period = 500;
  else if ((uint32_t)(now - lastActivityMs) < 250)              period = 100;
  else                                                          period = 0;

  if (period == 0) {
    if (!ledState) { ledState = true; ledWrite(true); }
  } else if ((uint32_t)(now - ledLastToggle) >= period) {
    ledLastToggle = now;
    ledState = !ledState;
    ledWrite(ledState);
  }
}

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
  while (!txDone) {}
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
  Serial.println(F("\r\n[TX OK]"));
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
  bool hasFix = false;
  if (gpsMode == GPS_SERIAL && gpsFixValid) hasFix = true;
  if (gpsMode == GPS_STATIC) hasFix = true;

  if (hasFix) {
    char lat[12], lon[13];
    aprsCoord(gpsLatRaw, gpsLatHemi, lat);
    aprsCoord(gpsLonRaw, gpsLonHemi, lon);
    
    if (wxMode != WX_OFF) {
      snprintf(out, outSize, "!%s%c%s_%s%s", lat, aprsSymTable, lon, wxData, beaconText);
    } else {
      snprintf(out, outSize, "!%s%c%s%c%s", lat, aprsSymTable, lon, aprsSymCode, beaconText);
    }
  } else {
    if (wxMode != WX_OFF) {
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
  Serial.print(F("\r\n[BEACON sent] "));
  Serial.println(payload);
  if (currentMode == COMMAND) Serial.print(F("cmd: "));
}

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
  while (i + 7 <= len && addrCount < 2 + MAX_PATH) {
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
  Serial.println(F("\r\n[DIGI repeated]"));
  if (currentMode == COMMAND) Serial.print(F("cmd: "));
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
  Serial.print(F("TX path set (")); Serial.print(txPathCount); Serial.println(F(" hops)."));
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
uint32_t frackMs  = DEF_FRACK_SEC * 1000UL;
uint8_t  retryMax = DEF_RETRY_MAX;
uint32_t t1At = 0; bool t1On = false;
uint32_t t3At = 0; bool t3On = false;
uint8_t  n2 = 0;

#define PACLEN_MAX 255
uint8_t paclen = DEF_PACLEN;
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
void printPeer(void) { Serial.print(peerCall); if (peerSSID) { Serial.print('-'); Serial.print(peerSSID); } }

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
  Serial.println(F("(connected - type to send, Ctrl+C for command mode)"));
}

void linkFailure(const char *why) {
  Serial.print(F("\r\n*** LINK FAILURE: ")); Serial.println(why);
  connReset();
  if (currentMode == CONVERSE) currentMode = COMMAND;
  Serial.print(F("cmd: "));
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
      Serial.print(F("\r\n<UI ")); Serial.print(srcCall);
      if (srcSSID) { Serial.print('-'); Serial.print(srcSSID); }
      Serial.print(F("> "));
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print(F("\r\ncmd: ")); else Serial.print(F("\r\n"));
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
      Serial.print(F("\r\n*** CONNECTED to ")); printPeer(); Serial.println();
      enterConnConverse();
    } else if (u == U_DISC) {
      sendU(U_UA, false, pf);
      if (linkState == LINK_CONN) Serial.print(F("\r\n*** DISCONNECTED\r\n"));
      connReset();
      if (currentMode == CONVERSE) currentMode = COMMAND;
      Serial.print(F("cmd: "));
    } else if (u == U_UA) {
      if (linkState == LINK_SETUP) {
        linkState = LINK_CONN; v_s = v_r = v_a = 0; n2 = 0; t1Stop(); t3Start();
        linkConnected = true;
        Serial.print(F("\r\n*** CONNECTED to ")); printPeer(); Serial.println();
        enterConnConverse();
      } else if (linkState == LINK_RELEASE) {
        Serial.print(F("\r\n*** DISCONNECTED\r\n")); connReset(); Serial.print(F("cmd: "));
      }
    } else if (u == U_DM) {
      if (linkState == LINK_SETUP)      { Serial.print(F("\r\n*** BUSY (connect refused)\r\n")); connReset(); Serial.print(F("cmd: ")); }
      else if (linkState == LINK_CONN)  { Serial.print(F("\r\n*** DISCONNECTED (by peer)\r\n")); connReset(); if (currentMode == CONVERSE) currentMode = COMMAND; Serial.print(F("cmd: ")); }
    } else if (u == U_FRMR) {
      if (linkState == LINK_CONN) { Serial.print(F("\r\n*** FRMR - resetting link\r\n")); sendU(U_DISC, true, true); linkState = LINK_RELEASE; n2 = 0; t1Start(); }
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
      Serial.print(F("\r\n"));
      for (uint16_t k = infoOff; k < len; k++) Serial.print((char)frame[k]);
      if (currentMode == COMMAND) Serial.print(F("\r\ncmd: "));
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
      if (linkState == LINK_RELEASE) { Serial.print(F("\r\n*** DISCONNECTED\r\n")); connReset(); if (currentMode == CONVERSE) currentMode = COMMAND; Serial.print(F("cmd: ")); }
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
  if (arg.length() == 0) { Serial.println(F("Usage: CONNECT <call[-ssid]>")); return; }
  if (linkState != LINK_DISC) { Serial.println(F("Busy - DISCONNECT first.")); return; }
  char c[7]; uint8_t s; parseCallSSID(arg, c, &s);
  setPeer(c, s);
  connPathCount = txPathCount;
  for (uint8_t i = 0; i < txPathCount; i++) { strncpy(connPathCall[i], txPathCalls[i], 7); connPathSSID[i] = txPathSSID[i]; }
  v_s = v_r = v_a = 0; n2 = 0; peerBusy = false; rejSent = false; txQHead = txQTail = txQCount = 0;
  linkState = LINK_SETUP;
  sendU(U_SABM, true, true);
  t1Start();
  Serial.print(F("*** Connecting to ")); printPeer(); Serial.println(F(" ..."));
}

void cmdDisconnect(void) {
  if (linkState == LINK_CONN || linkState == LINK_SETUP) {
    sendU(U_DISC, true, true);
    linkState = LINK_RELEASE; n2 = 0; t1Start();
    Serial.println(F("*** Disconnecting ..."));
    if (currentMode == CONVERSE) currentMode = COMMAND;
  } else Serial.println(F("Not connected."));
}

void cmdMheard(void) {
  if (heardCount == 0) { Serial.println(F("Nothing heard yet.")); return; }
  Serial.println(F("Stations heard:"));
  uint32_t now = millis();
  for (uint8_t i = 0; i < heardCount; i++) {
    Serial.print("  "); Serial.print(heardCall[i]);
    Serial.print("  ("); Serial.print((now - heardAt[i]) / 1000); Serial.println(F("s ago)"));
  }
}

void cmdStatus(void) {
  Serial.print(F("Link: "));
  if      (linkState == LINK_DISC)    Serial.println(F("DISCONNECTED"));
  else if (linkState == LINK_SETUP)  { Serial.print(F("CONNECTING to ")); printPeer(); Serial.println(); }
  else if (linkState == LINK_RELEASE) Serial.println(F("DISCONNECTING"));
  else {
    Serial.print(F("CONNECTED to ")); printPeer();
    Serial.print(F("  V(S)=")); Serial.print(v_s);
    Serial.print(F(" V(R)=")); Serial.print(v_r);
    Serial.print(F(" V(A)=")); Serial.print(v_a);
    Serial.print(F(" unacked=")); Serial.println(outstanding());
  }
}

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
  if (gpsMode != GPS_SERIAL) return;
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    if (c == '\r') continue;
    if (c == '\n') { if (nmeaLen > 0) { nmea[nmeaLen] = '\0'; parseNMEA(nmea); } nmeaLen = 0; }
    else if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
    else nmeaLen = 0;
  }
}

void cmdGpsStatus(void) {
  Serial.print(F("GPS Mode: "));
  if (gpsMode == GPS_OFF) Serial.println(F("OFF"));
  else if (gpsMode == GPS_STATIC) Serial.println(F("STATIC"));
  else Serial.println(F("SERIAL"));

  if (gpsMode == GPS_OFF) return;

  if (gpsMode == GPS_SERIAL) {
    Serial.print(F("Fix:  ")); Serial.println(gpsFixValid ? F("VALID") : F("no fix"));
    Serial.print(F("Sats: ")); Serial.println(gpsSats);
  } else {
    Serial.println(F("Fix:  STATIC"));
  }

  bool hasFix = (gpsMode == GPS_STATIC) || (gpsMode == GPS_SERIAL && gpsFixValid);
  if (hasFix) {
    Serial.print(F("Pos:  ")); Serial.print(gpsLatRaw); Serial.print(gpsLatHemi);
    Serial.print("  ");     Serial.print(gpsLonRaw); Serial.println(gpsLonHemi);
    char lat[12], lon[13];
    aprsCoord(gpsLatRaw, gpsLatHemi, lat);
    aprsCoord(gpsLonRaw, gpsLonHemi, lon);
    Serial.print(F("APRS: !")); Serial.print(lat); Serial.print(aprsSymTable);
    Serial.print(lon); Serial.println(aprsSymCode);
  }
}

void wxService(void) {
  if (wxMode != WX_SERIAL) return;
  while (wxSerial.available()) {
    char c = wxSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (wxSerialLen > 0) {
        wxSerialBuf[wxSerialLen] = '\0';
        strncpy(wxData, wxSerialBuf, sizeof(wxData) - 1);
        wxSerialLen = 0;
      }
    } else if (wxSerialLen < sizeof(wxSerialBuf) - 1) {
      wxSerialBuf[wxSerialLen++] = c;
    }
  }
}

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
    Serial.print(F("\r\n<RX> "));
    for (uint16_t i = 0; i + 1 < len; i++) {
      if (frame[i] == 0x03 && frame[i + 1] == 0xF0) {
        for (uint16_t j = i + 2; j < len; j++) Serial.print((char)frame[j]);
        foundText = true;
        break;
      }
    }
    if (!foundText) Serial.print(F("[Raw packet]"));
    if (currentMode == COMMAND) Serial.print(F("\r\ncmd: "));
    else                        Serial.print(F("\r\n"));
  }
}

void resetToDefaults() {
  memset(myCall, 0, sizeof(myCall)); strncpy(myCall, DEF_MYCALL, 6);
  mySSID = DEF_MYSSID;
  memset(beaconText, 0, sizeof(beaconText)); strncpy(beaconText, DEF_BEACON_TEXT, sizeof(beaconText) - 1);
  beaconEnabled = DEF_BEACON_EN;
  beaconInterval = DEF_BEACON_SEC * 1000UL;
  digiEnabled = DEF_DIGI_EN;
  digiMode = (DigiMode)DEF_DIGI_MODE;
  wideMax = DEF_WIDE_MAX;
  aliasEnabled = DEF_ALIAS_EN;
  memset(myAlias, 0, sizeof(myAlias)); strncpy(myAlias, DEF_ALIAS, 6);
  myAliasSSID = DEF_ALIAS_SSID;
  memset(txPathCalls[0], 0, 7); strncpy(txPathCalls[0], DEF_TX_PATH_CALL1, 6); txPathSSID[0] = DEF_TX_PATH_SSID1;
  memset(txPathCalls[1], 0, 7); strncpy(txPathCalls[1], DEF_TX_PATH_CALL2, 6); txPathSSID[1] = DEF_TX_PATH_SSID2;
  txPathCount = DEF_TX_PATH_CNT;
  frackMs = DEF_FRACK_SEC * 1000UL;
  retryMax = DEF_RETRY_MAX;
  paclen = DEF_PACLEN;
  gpsMode = (GpsMode)DEF_GPS_MODE;
  gpsBaud = DEF_GPS_BAUD;
  aprsSymTable = DEF_APRS_SYM_TBL;
  aprsSymCode = DEF_APRS_SYM_CODE;
  wxMode = (WxMode)DEF_WX_MODE;
  memset(wxData, 0, sizeof(wxData)); strncpy(wxData, DEF_WX_DATA, sizeof(wxData) - 1);
  memset(gpsLatRaw, 0, sizeof(gpsLatRaw)); strncpy(gpsLatRaw, DEF_GPS_LAT, sizeof(gpsLatRaw) - 1);
  gpsLatHemi = DEF_GPS_LATH;
  memset(gpsLonRaw, 0, sizeof(gpsLonRaw)); strncpy(gpsLonRaw, DEF_GPS_LON, sizeof(gpsLonRaw) - 1);
  gpsLonHemi = DEF_GPS_LONH;
}

// Safely loads byte-by-byte into SRAM to PREVENT Misaligned HardFaults
void readFlashStruct(ConfigStruct *dest) {
  uint8_t *src = (uint8_t *)FLASH_CONFIG_ADDR;
  uint8_t *dst = (uint8_t *)dest;
  for (size_t i = 0; i < sizeof(ConfigStruct); i++) {
    dst[i] = src[i];
  }
}

// Writes 16-bit half-words manually
void writeFlashStruct(ConfigStruct *src_struct) {
  SampleTimer->pause();
  PwmTimer->pause();
  noInterrupts(); // Safe to turn off interrupts here because we aren't using HAL delays

  // Unlock Flash
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  while (FLASH->SR & FLASH_SR_BSY);

  // Erase Page
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = FLASH_CONFIG_ADDR;
  FLASH->CR |= FLASH_CR_STRT;
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_PER;

  // Program Half-Words
  FLASH->CR |= FLASH_CR_PG;
  uint16_t *data = (uint16_t *)src_struct;
  uint32_t halfwords = (sizeof(ConfigStruct) + 1) / 2;
  for (uint32_t i = 0; i < halfwords; i++) {
    *(volatile uint16_t*)(FLASH_CONFIG_ADDR + (i * 2)) = data[i];
    while (FLASH->SR & FLASH_SR_BSY);
  }
  FLASH->CR &= ~FLASH_CR_PG;
  
  // Lock Flash
  FLASH->CR |= FLASH_CR_LOCK;

  interrupts();
  PwmTimer->resume();
  SampleTimer->resume();
}

void loadConfig() {
  ConfigStruct c;
  readFlashStruct(&c); // Byte-level extraction prevents alignment crash
  
  if (c.magic == FLASH_MAGIC) {
    memcpy(myCall, c.myCall, sizeof(myCall));
    mySSID = c.mySSID;
    memcpy(beaconText, c.beaconText, sizeof(beaconText));
    beaconEnabled = c.beaconEnabled;
    beaconInterval = c.beaconInterval;
    digiEnabled = c.digiEnabled;
    digiMode = (DigiMode)c.digiMode;
    wideMax = c.wideMax;
    aliasEnabled = c.aliasEnabled;
    memcpy(myAlias, c.myAlias, sizeof(myAlias));
    myAliasSSID = c.myAliasSSID;
    memcpy(txPathCalls, c.txPathCalls, sizeof(txPathCalls));
    memcpy(txPathSSID, c.txPathSSID, sizeof(txPathSSID));
    
    txPathCount = c.txPathCount;
    if (txPathCount > MAX_PATH) txPathCount = MAX_PATH; 
    
    frackMs = c.frackMs;
    retryMax = c.retryMax;
    if (retryMax > 30) retryMax = 10;
    paclen = c.paclen;
    if (paclen > PACLEN_MAX) paclen = PACLEN_MAX; 
    
    gpsMode = (GpsMode)c.gpsMode;
    gpsBaud = c.gpsBaud;
    aprsSymTable = c.aprsSymTable;
    aprsSymCode = c.aprsSymCode;
    wxMode = (WxMode)c.wxMode;
    memcpy(wxData, c.wxData, sizeof(wxData));
    memcpy(gpsLatRaw, c.gpsLatRaw, sizeof(gpsLatRaw));
    gpsLatHemi = c.gpsLatHemi;
    memcpy(gpsLonRaw, c.gpsLonRaw, sizeof(gpsLonRaw));
    gpsLonHemi = c.gpsLonHemi;
  } else {
    // Magic missing: fresh flash. Load defaults. DO NOT auto-save so USB isn't hung.
    resetToDefaults();
  }
}

void saveConfig() {
  ConfigStruct c;
  memset(&c, 0, sizeof(ConfigStruct)); // Ensure zero padding
  
  c.magic = FLASH_MAGIC;
  memcpy(c.myCall, myCall, sizeof(myCall));
  c.mySSID = mySSID;
  memcpy(c.beaconText, beaconText, sizeof(beaconText));
  c.beaconEnabled = beaconEnabled;
  c.beaconInterval = beaconInterval;
  c.digiEnabled = digiEnabled;
  c.digiMode = (uint8_t)digiMode;
  c.wideMax = wideMax;
  c.aliasEnabled = aliasEnabled;
  memcpy(c.myAlias, myAlias, sizeof(myAlias));
  c.myAliasSSID = myAliasSSID;
  memcpy(c.txPathCalls, txPathCalls, sizeof(txPathCalls));
  memcpy(c.txPathSSID, txPathSSID, sizeof(txPathSSID));
  c.txPathCount = txPathCount;
  c.frackMs = frackMs;
  c.retryMax = retryMax;
  c.paclen = paclen;
  c.gpsMode = (uint8_t)gpsMode;
  c.gpsBaud = gpsBaud;
  c.aprsSymTable = aprsSymTable;
  c.aprsSymCode = aprsSymCode;
  c.wxMode = (uint8_t)wxMode;
  memcpy(c.wxData, wxData, sizeof(wxData));
  memcpy(c.gpsLatRaw, gpsLatRaw, sizeof(gpsLatRaw));
  c.gpsLatHemi = gpsLatHemi;
  memcpy(c.gpsLonRaw, gpsLonRaw, sizeof(gpsLonRaw));
  c.gpsLonHemi = gpsLonHemi;
  
  writeFlashStruct(&c);
  Serial.println(F("Settings successfully saved to Flash."));
}

void formatFlash() {
  ConfigStruct c;
  memset(&c, 0, sizeof(ConfigStruct)); // Zero everything including magic
  writeFlashStruct(&c);
  resetToDefaults();
  Serial.println(F("Flash page wiped. Defaults loaded. Type SAVE to commit."));
}

void showSavedConfig() {
  ConfigStruct c;
  readFlashStruct(&c);
  if (c.magic != FLASH_MAGIC) {
    Serial.println(F("Flash storage is empty. Currently using memory defaults."));
    Serial.println(F("Type SAVE to commit to flash."));
    return;
  }
  Serial.println(F("--- Saved Configuration ---"));
  Serial.print(F("MYCALL:     ")); Serial.print(c.myCall); if(c.mySSID) { Serial.print('-'); Serial.print(c.mySSID); } Serial.println();
  Serial.print(F("BTEXT:      ")); Serial.println(c.beaconText);
  Serial.print(F("BEACON:     ")); if (c.beaconEnabled) { Serial.print(F("EVERY ")); Serial.print(c.beaconInterval / 1000); Serial.println(F("s")); } else { Serial.println(F("OFF")); }
  Serial.print(F("DIGI:       ")); Serial.println(c.digiEnabled ? F("ON") : F("OFF"));
  
  Serial.print(F("GPS MODE:   ")); Serial.println(c.gpsMode == GPS_OFF ? F("OFF") : (c.gpsMode == GPS_STATIC ? F("STATIC") : F("SERIAL")));
  Serial.print(F("GPS DATA:   ")); Serial.print(c.gpsLatRaw); Serial.print(c.gpsLatHemi); Serial.print(F(" ")); Serial.print(c.gpsLonRaw); Serial.println(c.gpsLonHemi);
  Serial.print(F("GPS BAUD:   ")); Serial.println(c.gpsBaud);
  
  Serial.print(F("WX MODE:    ")); Serial.println(c.wxMode == WX_OFF ? F("OFF") : (c.wxMode == WX_MANUAL ? F("MANUAL") : F("SERIAL")));
  Serial.print(F("WX DATA:    ")); Serial.println(c.wxData);
  Serial.println(F("---------------------------"));
}

void setup() {
  Serial.begin(115200);

  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  ledWrite(true);

  loadConfig();

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

  // Allow USB to enumerate properly
  while (!Serial && millis() < 3000);

  if (gpsMode == GPS_SERIAL) gpsBegin();
  if (wxMode == WX_SERIAL) wxSerial.begin(9600);

  Serial.println(F("\r\n*** STM32 Blue Pill TNC (AFSK1200) ***"));
  ConfigStruct check;
  readFlashStruct(&check);
  if (check.magic != FLASH_MAGIC) {
    Serial.println(F(">> FIRST BOOT DETECTED: Defaults loaded into memory."));
    Serial.println(F(">> Type 'SAVE' to commit them to Flash."));
  }
  Serial.println(F("Type HELP for commands."));
  Serial.print(F("cmd: "));
}

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

    if (currentMode == KISS) {
      if (b == FEND) {
        if (inKissFrame && kissBufferIndex > 1) {
          uint8_t command = kissBuffer[0] & 0x0F;
          if (command == 0x00) {
            sendAX25Frame(kissBuffer + 1, kissBufferIndex - 1);
          } else if (command == 0x0F) {
            currentMode = COMMAND;
            linkConnected = false;
            Serial.println(F("\r\nExited KISS Mode."));
            Serial.print(F("cmd: "));
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

    if (b == 0x03 && currentMode == CONVERSE) {
      currentMode = COMMAND;
      inputLen = 0;
      Serial.print(F("\r\ncmd: "));
      continue;
    }

    if (b != '\r' && b != '\n' && b != 0x03) Serial.print((char)b);

    if (b == '\n' || b == '\r') {
      if (inputLen == 0) {
        if (currentMode == COMMAND) Serial.print(F("\r\ncmd: "));
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
              Serial.println(F("[link busy]"));
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
          Serial.print(F("MYCALL: ")); Serial.println(myCall);
        } else if (cmd.startsWith("MYSSID ")) {
          mySSID = (uint8_t) cmd.substring(7).toInt() & 0x0F;
          Serial.print(F("MYSSID: ")); Serial.println(mySSID);
        } else if (cmd.startsWith("BTEXT ")) {
          String t = raw.substring(6); t.trim();
          t.toCharArray(beaconText, sizeof(beaconText));
          Serial.print(F("BTEXT set: ")); Serial.println(beaconText);
        } else if (cmd.startsWith("BEACON EVERY ")) {
          uint32_t s = (uint32_t) cmd.substring(13).toInt();
          if (s == 0) { beaconEnabled = false; Serial.println(F("Beacon off.")); }
          else {
            beaconInterval = s * 1000UL;
            beaconEnabled  = true;
            lastBeaconMs   = millis();
            Serial.print(F("Beacon every ")); Serial.print(s); Serial.println(F("s."));
          }
        } else if (cmd == "BEACON OFF") {
          beaconEnabled = false;
          Serial.println(F("Beacon off."));
        } else if (cmd == "BEACON") {
          sendBeacon();
        } else if (cmd == "DIGI ON") {
          digiEnabled = true;  Serial.println(F("Digi ON."));
        } else if (cmd == "DIGI OFF") {
          digiEnabled = false; Serial.println(F("Digi OFF."));
        } else if (cmd == "DIGI FILL") {
          digiMode = DIGI_FILL; Serial.println(F("Digi: FILL"));
        } else if (cmd == "DIGI WIDE") {
          digiMode = DIGI_WIDE; Serial.println(F("Digi: WIDE"));
        } else if (cmd.startsWith("WIDEMAX ")) {
          wideMax = (uint8_t) constrain(cmd.substring(8).toInt(), 1, 7);
          Serial.print(F("WIDEMAX=")); Serial.println(wideMax);
        } else if (cmd.startsWith("MYALIAS ")) {
          String a = cmd.substring(8); a.trim();
          if (a == "OFF") { aliasEnabled = false; Serial.println(F("Alias off.")); }
          else { parseCallSSID(a, myAlias, &myAliasSSID); aliasEnabled = true; Serial.println(F("Alias set.")); }
        } else if (cmd == "PATH" || cmd == "PATH OFF") {
          txPathCount = 0;
          Serial.println(F("Path cleared."));
        } else if (cmd.startsWith("PATH ")) {
          setTxPath(cmd.substring(5));
        } else if (cmd.startsWith("CONNECT ") || cmd.startsWith("C ")) {
          cmdConnect(cmd.substring(cmd.indexOf(' ') + 1));
        } else if (cmd == "DISCONNECT" || cmd == "D" || cmd == "BYE") {
          cmdDisconnect();
        } else if (cmd == "MHEARD" || cmd == "MH") {
          cmdMheard();
        } else if (cmd == "MONITOR ON") {
          monitorOn = true;  Serial.println(F("Monitor ON."));
        } else if (cmd == "MONITOR OFF") {
          monitorOn = false; Serial.println(F("Monitor OFF."));
        } else if (cmd == "STATUS" || cmd == "CS") {
          cmdStatus();
        } else if (cmd.startsWith("FRACK ")) {
          frackMs = (uint32_t) constrain(cmd.substring(6).toInt(), 1, 60) * 1000UL;
          Serial.print(F("FRACK=")); Serial.println(frackMs / 1000);
        } else if (cmd.startsWith("RETRY ")) {
          retryMax = (uint8_t) constrain(cmd.substring(6).toInt(), 1, 30);
          Serial.print(F("RETRY=")); Serial.println(retryMax);
        } else if (cmd.startsWith("PACLEN ")) {
          paclen = (uint8_t) constrain(cmd.substring(7).toInt(), 1, PACLEN_MAX);
          Serial.print(F("PACLEN=")); Serial.println(paclen);
        } else if (cmd == "GPS ON" || cmd == "GPS SERIAL") {
          gpsMode = GPS_SERIAL; gpsBegin(); Serial.println(F("GPS SERIAL."));
        } else if (cmd == "GPS OFF") {
          gpsMode = GPS_OFF; gpsEnd(); Serial.println(F("GPS OFF."));
        } else if (cmd == "GPS STATIC") {
          gpsMode = GPS_STATIC; gpsEnd(); Serial.println(F("GPS STATIC."));
        } else if (cmd.startsWith("GPS SET ")) {
          String args = raw.substring(8); 
          args.trim();
          int spaceIdx = args.indexOf(' ');
          if (spaceIdx > 0) {
            String latStr = args.substring(0, spaceIdx);
            String lonStr = args.substring(spaceIdx + 1);
            latStr.trim(); lonStr.trim();
            gpsLatHemi = latStr.charAt(latStr.length() - 1);
            memset(gpsLatRaw, 0, sizeof(gpsLatRaw));
            latStr.substring(0, latStr.length() - 1).toCharArray(gpsLatRaw, sizeof(gpsLatRaw));
            gpsLonHemi = lonStr.charAt(lonStr.length() - 1);
            memset(gpsLonRaw, 0, sizeof(gpsLonRaw));
            lonStr.substring(0, lonStr.length() - 1).toCharArray(gpsLonRaw, sizeof(gpsLonRaw));
            Serial.println(F("Static GPS set."));
          } else {
            Serial.println(F("Format: GPS SET <lat><hemi> <lon><hemi> (e.g. GPS SET 4523.12N 12100.34W)"));
          }
        } else if (cmd.startsWith("GPS BAUD ")) {
          gpsBaud = (uint32_t) cmd.substring(9).toInt();
          if (gpsMode == GPS_SERIAL) { gpsEnd(); gpsBegin(); }
          Serial.print(F("GPS Baud: ")); Serial.println(gpsBaud);
        } else if (cmd == "GPS" || cmd == "GPS STATUS") {
          cmdGpsStatus();
        } else if (cmd.startsWith("SYMBOL ")) {
          String s = raw.substring(7); s.trim();
          if (s.length() >= 2) { aprsSymTable = s.charAt(0); aprsSymCode = s.charAt(1); Serial.println(F("Symbol set.")); }
        } else if (cmd == "WX OFF") {
          wxMode = WX_OFF; Serial.println(F("WX OFF."));
        } else if (cmd == "WX MANUAL") {
          wxMode = WX_MANUAL; Serial.println(F("WX MANUAL."));
        } else if (cmd == "WX SERIAL") {
          wxMode = WX_SERIAL; wxSerial.begin(9600); Serial.println(F("WX SERIAL."));
        } else if (cmd.startsWith("WX SET ")) {
          String w = raw.substring(7);
          w.trim();
          memset(wxData, 0, sizeof(wxData));
          w.toCharArray(wxData, sizeof(wxData));
          Serial.print(F("WX data set: "));
          Serial.println(wxData);
        } else if (cmd == "SAVE") {
          saveConfig();
        } else if (cmd == "FORMAT") {
          formatFlash();
        } else if (cmd == "CONFIG" || cmd == "SHOW") {
          showSavedConfig();
        } else if (cmd == "CONV") {
          currentMode = CONVERSE;
          Serial.println(F("Converse mode (Ctrl+C to exit)"));
        } else if (cmd == "KISS ON") {
          currentMode = KISS;
          linkConnected = true;
          inKissFrame = false; kissBufferIndex = 0;
          Serial.println(F("KISS Mode active."));
        } else if (cmd == "HELP") {
          Serial.println(F("Commands:"));
          Serial.println(F("  MYCALL <c> / MYSSID <n> - Set callsign and SSID"));
          Serial.println(F("  PATH <a,b,..>|OFF   - TX digi path (e.g. WIDE1-1)"));
          Serial.println(F("  CONNECT <c>        - Connect to station (C)"));
          Serial.println(F("  DISCONNECT         - Close link (D, BYE)"));
          Serial.println(F("  STATUS             - Show link state (CS)"));
          Serial.println(F("  FRACK / RETRY / PACLEN - Link parameters"));
          Serial.println(F("  GPS OFF|SERIAL|STATIC - GPS Operating Mode"));
          Serial.println(F("  GPS SET <lat> <lon> - Set static pos (e.g. 4523.12N 12100.34W)"));
          Serial.println(F("  GPS BAUD <n>       - GPS serial speed"));
          Serial.println(F("  GPS                - Show GPS status/location"));
          Serial.println(F("  SYMBOL <tbl><code> - APRS symbol (e.g. />)"));
          Serial.println(F("  WX OFF|MANUAL|SERIAL - Weather station mode"));
          Serial.println(F("  WX SET <data>      - Set static WX telemetry"));
          Serial.println(F("  MHEARD             - Stations heard (MH)"));
          Serial.println(F("  MONITOR ON|OFF     - Show/hide UI frames"));
          Serial.println(F("  BTEXT <text>       - Set beacon text"));
          Serial.println(F("  BEACON             - Send beacon once"));
          Serial.println(F("  BEACON EVERY <sec> - Auto-beacon (0 = off)"));
          Serial.println(F("  DIGI ON|OFF        - Digipeater state"));
          Serial.println(F("  DIGI FILL|WIDE     - Fill-in or WIDEn-N mode"));
          Serial.println(F("  WIDEMAX <1-7>      - Max N in WIDE mode"));
          Serial.println(F("  MYALIAS <c>|OFF    - Extra exact-match alias"));
          Serial.println(F("  SAVE               - Save settings to Flash"));
          Serial.println(F("  CONFIG             - Show saved settings (SHOW)"));
          Serial.println(F("  FORMAT             - Erase flash & load Defaults"));
          Serial.println(F("  CONV               - Converse mode"));
          Serial.println(F("  KISS ON            - KISS TNC mode"));
        } else {
          Serial.println(F("ERROR"));
        }

        if (currentMode == COMMAND) Serial.print(F("cmd: "));
      }
      inputLen = 0;
    } else {
      if (inputLen < sizeof(inputBuffer) - 1) inputBuffer[inputLen++] = b;
    }
  }
}