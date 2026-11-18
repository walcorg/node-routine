/*
  ============================================================================
  WALCORG TERRAFORMING CORP. - Node v4.5 (data acquisition model)
  Unified firmware for the Arduino Nano (ATmega328P)          Version 0.2
  ============================================================================

  Merges the previous routines into a single sketch:
    - rutine_verify_photo_zero_v1/v2   -> command 'G' (darkness check)
    - rutine_photo_sampling_v1/v2      -> command 'H' (single LED + reading)
    - password_sha-256 / leds_codif.   -> command 'K' (byte on 8 LEDs + reading)
    - rutine_modules_control_v1 (B,C,D)-> commands 'B', 'C', 'D'
    - rutine_sterelization_uv (E,F)    -> commands 'E', 'F'
  Everything runs over I2C (slave); the serial port is no longer used for control.

  ---------------------------------------------------------------------------
  HARDWARE (rigid PCB v4.5)
  ---------------------------------------------------------------------------
    TLC59401 (16 constant-current sink channels):
      D3  GSCLK   D9  XLAT   D10 BLANK   D11 SIN   D13 SCLK  (driven by the library)
      D12 MODE  (VPRG: LOW = grayscale mode)
      A6  XERR  (open drain, 10k pull-up; LOW = open LED or over-temperature)
      Channels: 0-7   LED_K0..K7   data LEDs (1 bit each, LSB = LED_K0)
                8-11  LED_K8..K11  calibration LEDs
                12-15 UV_K12..K15  UVC LEDs (anode on +V_UV, 9 V)
    CD74HC4067 (16:1 analog multiplexer):
      D4-D7 S0-S3   A0 MUX_SIG (COMMON)   E tied to GND
      Inputs: 0-7  SENS_B0..B7 (backscatter, J1)
              8-15 SENS_T0..T7 (transmission, J2)
    A3  RANGE_SEL  output LOW = normal range (~9.1k)
                   input (high Z) = sensitive range (100k). NEVER drive it HIGH.
    A2  UV_EN      enables the MT3608 boost converter (+V_UV). LOW = off.
    A1  INT_EXP1   interrupt from expansion modules (active LOW)
    D2  ADDR0, D8 ADDR1  solder jumpers to GND -> address 0x10..0x13
    A4/A5 SDA/SCL  I2C bus to the shield (4.7k pull-ups on the shield)

  Required library: "Tlc5940" (Alex Leone; Paul Stoffregen fork).
  With its default configuration (NUM_TLCS 1, VPRG_ENABLED 0, TLC_SPI) the
  pins match this board. It uses Timer1 and Timer2: do not use analogWrite()
  on D3, D9, D10, D11, nor tone().

  ---------------------------------------------------------------------------
  I2C PROTOCOL  (address 0x10 + ADDR1*2 + ADDR0; closed jumper = 1)
  ---------------------------------------------------------------------------
  The master writes [command, arguments...] (32 bytes max).
  After any command (except 'R'), a read returns the STATUS:
     8 bytes: [state, flags, last_cmd, error, fw_version,
               uv_sec_remaining_L, uv_sec_remaining_H, i2c_address]
  'R',page selects a DATA page; the next read returns
     32 bytes = 16 little-endian uint16 values (0-1023):
       page 0      last measurement (16 sensors, index = mux channel)
       page 1      calibration: dark baseline (all LEDs off)
       page 2..13  calibration: LED 0..11 on alone
       page 14     current configuration (parameters 0..5, see 'P')
  '?' switches back to STATUS mode without running anything.

  Commands:
   'G'                    Darkness check: turns the LEDs off and measures the
                          16 sensors. Sets DARK_ALERT if any reading > threshold.
   'H', n                 Turns on only LED n (0-11), measures, then turns it off.
   'K', byte              Shows the byte on LEDs 0-7 (bit i -> LED i), waits,
                          measures and keeps the pattern on.
                          Building block for encoding text one character at a time.
   'M', maskL, maskH      Turns on the visible LEDs given by the 12-bit mask.
   'X', l0L, l0H, ...     Laura Encoding pattern: sets each visible LED 0..11 to its own
        ... l11L, l11H    12-bit intensity (0-4095), waits, measures the 16 sensors and
                          turns the LEDs off. Exactly 12 levels (25-byte frame).
                          Each sensor averages 32 samples (~3.6 ms) to smooth the PWM.
                          Used for optical challenges (see "Laura Encoding" below).
   'S'                    Measures the 16 sensors with the LEDs as they are.
   'O'                    Turns all visible LEDs off.
   'B', vL, vH, text...   Plays the text on LEDs 0-7, v = ms per character
                          (no measurement; for demos). Up to 29 characters.
   'C'                    Full calibration: dark baseline + each LED 0-11 alone.
                          Results in pages 1-13.
   'E' [, sL, sH]         UVC disinfection for s seconds (default 300,
                          max 1800). Fast blinking when turning on and off.
   'F'                    Stops UVC disinfection.
   'D'                    Safe mode: turns EVERYTHING off immediately
                          (LEDs, UVC, boost converter).
   'L', r                 Measurement range: 0 normal, 1 sensitive.
   'P', id, vL, vH        Sets a parameter:
                            0 LED settle time (ms, default 30)
                            1 dark threshold (0-1023, default 20)
                            2 visible LED intensity (0-4095, default 4095)
                            3 UVC intensity (0-4095, default 4095)
                            4 samples averaged per sensor (1-16, default 4)
                            5 default UVC duration (s, default 300)
  'F' and 'D' are served even while another operation is running.

  States: 0 IDLE, 1 BUSY, 2 UVC (ramping on / on / ramping off), 3 PLAYING
  Flags (bits): 0 DATA_READY  1 DARK_ALERT  2 XERR  3 UV_ACTIVE
                4 SENSITIVE_RANGE  5 INT_EXP  6 WATCHDOG_RESET
  Laura Encoding (Codificado de Laura):
  The culture between the LEDs and the sensors acts as a 16x12 transfer matrix M:
  reading = M * levels + dark + noise. 'C' measures M (pages 2..13 minus page 1)
  and 'X' answers a challenge with an arbitrary intensity pattern. Verification
  happens on the server, which keeps M; this firmware only performs the physics.
  Specification: github.com/walcorg/clorela-os/blob/main/docs/laura-encoding.md

  Errors: 0 none, 1 unknown command, 2 invalid argument,
          3 busy (a command was already pending), 4 blocked by active UVC,
          5 aborted by 'F' or 'D'
  ============================================================================
*/

#include <Wire.h>
#include <Tlc5940.h>
#include <avr/wdt.h>
#include <string.h>

// ----------------------------- Build options ---------------------------------
#define FW_VERSION   0x02   // 0x02: adds 'X' (Laura Encoding pattern)
#define USE_WATCHDOG 1   // Set to 0 if the Nano has the old bootloader
                         // (with that bootloader a watchdog reset leaves it
                         //  stuck in a reset loop).
#define DEBUG        0   // 1 = debug messages on Serial at 115200 (D0/D1 are free)

#if DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// ----------------------------- Pins ------------------------------------------
const uint8_t PIN_ADDR0     = 2;
const uint8_t PIN_ADDR1     = 8;
const uint8_t PIN_MUX_S[4]  = {4, 5, 6, 7};
const uint8_t PIN_TLC_MODE  = 12;
const uint8_t PIN_MUX_SIG   = A0;
const uint8_t PIN_INT_EXP   = A1;
const uint8_t PIN_UV_EN     = A2;
const uint8_t PIN_RANGE_SEL = A3;
const uint8_t PIN_TLC_XERR  = A6;   // analog only

// ----------------------------- Channel map -----------------------------------
const uint8_t CH_DATA0     = 0;    // data LEDs 0..7
const uint8_t NUM_VISIBLE  = 12;   // channels 0..11 (data + calibration)
const uint8_t CH_UV0       = 12;   // UVC 12..15
const uint8_t NUM_UV       = 4;
const uint8_t NUM_SENS     = 16;

// ----------------------------- Safety limits ---------------------------------
const uint16_t UV_HARD_MAX_SEC = 1800;  // 30 min, cannot be exceeded over I2C
const uint16_t UV_BLINK_MS     = 30;
const uint8_t  UV_BLINK_COUNT  = 10;
const uint16_t UV_BOOST_START_MS = 10;  // MT3608 start-up before the first blink
// 'X' uses intermediate intensities, which the TLC59401 makes with PWM at ~1.95 kHz
// (512 us period). One analogRead takes ~112 us, so 32 samples span ~3.6 ms
// (~7 PWM periods) and the average follows the mean light, not the PWM phase.
const uint8_t  PATTERN_SAMPLES = 32;

// ----------------------------- Codes -----------------------------------------
enum : uint8_t { ST_IDLE = 0, ST_BUSY = 1, ST_UV = 2, ST_PLAY = 3 };
enum : uint8_t { ERR_NONE = 0, ERR_UNKNOWN = 1, ERR_ARG = 2, ERR_BUSY = 3,
                 ERR_UV_ACTIVE = 4, ERR_ABORTED = 5 };
enum : uint8_t { F_DATA = 0, F_DARK = 1, F_XERR = 2, F_UV = 3,
                 F_RANGE = 4, F_INT = 5, F_WDT = 6 };
enum : uint8_t { RESP_STATUS = 0, RESP_DATA = 1 };
enum : uint8_t { UV_OFF = 0, UV_RAMP_ON, UV_ON, UV_RAMP_OFF };

// ----------------------------- Configuration ---------------------------------
struct Config {
  uint16_t ledSettleMs   = 30;    // photoresistors / phototransistors
  uint16_t darkThreshold = 20;
  uint16_t visLevel      = 4095;
  uint16_t uvLevel       = 4095;
  uint16_t samples       = 4;
  uint16_t uvDefaultSec  = 300;
} cfg;
const uint8_t NUM_PARAMS = 6;

// ----------------------------- Data ------------------------------------------
uint16_t frame[NUM_SENS];                     // last measurement
uint16_t calib[1 + NUM_VISIBLE][NUM_SENS];    // row 0 dark, rows 1..12 LEDs
uint16_t visibleMask = 0;                     // visible LEDs currently on
bool     rangeSensitive = false;

// ----------------------------- Shared with the I2C ISR -----------------------
volatile uint8_t  rxBuf[32];
volatile uint8_t  rxLen = 0;
volatile bool     cmdPending = false;
volatile bool     abortReq = false;
volatile uint8_t  respMode = RESP_STATUS;
volatile uint8_t  txData[32];
volatile uint8_t  state = ST_IDLE;
volatile uint8_t  flags = 0;
volatile uint8_t  lastCmd = 0;
volatile uint8_t  lastErr = ERR_NONE;
volatile uint16_t uvRemaining = 0;
uint8_t i2cAddr = 0x10;

// ----------------------------- UVC -------------------------------------------
uint8_t       uvPhase = UV_OFF;
uint8_t       uvToggles = 0;
bool          uvLit = false;
unsigned long uvT0 = 0;          // start of the current phase / last blink
unsigned long uvDurationMs = 0;

// ----------------------------- Text playback ---------------------------------
bool          playing = false;
uint8_t       playBuf[29];
uint8_t       playLen = 0, playIdx = 0;
uint16_t      playMs = 100;
unsigned long playNext = 0;

// =============================================================================
//  Helpers
// =============================================================================
static inline void setFlag(uint8_t bit, bool on) {
  uint8_t f = flags;
  if (on) f |= (1 << bit); else f &= ~(1 << bit);
  flags = f;                        // single-byte write: atomic
}

static inline void petDog() {
#if USE_WATCHDOG
  wdt_reset();
#endif
}

// Waits ms while feeding the watchdog and checking for abort. Returns false if aborted.
bool waitMs(uint16_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    petDog();
    if (abortReq) return false;
  }
  return true;
}

// Sends the data to the TLC (waits if the previous transfer has not been latched yet).
void tlcCommit() {
  while (Tlc.update()) { petDog(); }
}

void applyVisible(uint16_t mask) {
  mask &= 0x0FFF;                   // channels 0..11 only; UVC channels are never touched here
  for (uint8_t ch = 0; ch < NUM_VISIBLE; ch++) {
    Tlc.set(ch, (mask & (1u << ch)) ? cfg.visLevel : 0);
  }
  visibleMask = mask;
  tlcCommit();
}

void applyUV(bool on) {
  for (uint8_t i = 0; i < NUM_UV; i++) Tlc.set(CH_UV0 + i, on ? cfg.uvLevel : 0);
  uvLit = on;
  tlcCommit();
}

void setRange(bool sensitive) {
  if (sensitive) {
    pinMode(PIN_RANGE_SEL, INPUT);          // high impedance -> 100k only
  } else {
    digitalWrite(PIN_RANGE_SEL, LOW);       // LOW first so the pin never goes HIGH
    pinMode(PIN_RANGE_SEL, OUTPUT);         // 10k in parallel -> ~9.1k
  }
  rangeSensitive = sensitive;
  setFlag(F_RANGE, sensitive);
}

void setMux(uint8_t ch) {
  for (uint8_t b = 0; b < 4; b++) digitalWrite(PIN_MUX_S[b], (ch >> b) & 1);
}

// Measures the 16 sensors. The first conversion of each channel is discarded
// because the ADC sample-and-hold capacitor charges slowly from high-Z sources.
bool readAllN(uint16_t *out, uint8_t n) {
  for (uint8_t ch = 0; ch < NUM_SENS; ch++) {
    setMux(ch);
    if (rangeSensitive) { if (!waitMs(5)) return false; }   // 100k * 10nF -> 5 tau
    else delayMicroseconds(300);
    analogRead(PIN_MUX_SIG);
    uint32_t acc = 0;
    for (uint8_t i = 0; i < n; i++) acc += analogRead(PIN_MUX_SIG);
    out[ch] = acc / n;
    petDog();
    if (abortReq) return false;
  }
  return true;
}

bool readAll(uint16_t *out) {
  return readAllN(out, constrain(cfg.samples, 1, 16));
}

// Measures into frame[] with n samples per sensor and sets DATA_READY.
bool measureFrameN(uint8_t n) {
  setFlag(F_DATA, false);
  if (!readAllN(frame, n)) return false;
  setFlag(F_DATA, true);
  return true;
}

bool measureFrame() {
  return measureFrameN(constrain(cfg.samples, 1, 16));
}

// =============================================================================
//  UVC: non-blocking state machine
// =============================================================================
void uvStart(uint16_t sec) {
  playing = false;
  applyVisible(0);                          // USB current budget
  digitalWrite(PIN_UV_EN, HIGH);            // start the boost converter
  uvDurationMs = (unsigned long)sec * 1000UL;
  uvPhase = UV_RAMP_ON;
  uvToggles = 0;
  uvT0 = millis() + UV_BOOST_START_MS;      // first blink after boost start-up
  uvRemaining = sec;
  setFlag(F_UV, true);
  DBG(F("UV start ")); DBGLN(sec);
}

void uvStop() {
  if (uvPhase == UV_OFF || uvPhase == UV_RAMP_OFF) return;
  uvPhase = UV_RAMP_OFF;
  uvToggles = 0;
  uvT0 = millis();
  DBGLN(F("UV stop"));
}

void uvKill() {                              // immediate shutdown
  applyUV(false);
  digitalWrite(PIN_UV_EN, LOW);
  uvPhase = UV_OFF;
  uvRemaining = 0;
  setFlag(F_UV, false);
}

void uvService() {
  unsigned long now = millis();
  switch (uvPhase) {
    case UV_RAMP_ON:
      if ((long)(now - uvT0) >= 0) {
        applyUV(!uvLit);
        uvT0 = now + UV_BLINK_MS;
        if (++uvToggles >= 2 * UV_BLINK_COUNT) {
          applyUV(true);                    // stays on steadily
          uvPhase = UV_ON;
          uvT0 = now;
        }
      }
      break;
    case UV_ON: {
      unsigned long el = now - uvT0;
      if (el >= uvDurationMs) { uvStop(); break; }
      uvRemaining = (uvDurationMs - el + 999UL) / 1000UL;
      break;
    }
    case UV_RAMP_OFF:
      if ((long)(now - uvT0) >= 0) {
        applyUV(!uvLit);
        uvT0 = now + UV_BLINK_MS;
        if (++uvToggles >= 2 * UV_BLINK_COUNT) uvKill();
      }
      break;
    default: break;
  }
}

// =============================================================================
//  Text playback on the 8 data LEDs
// =============================================================================
void playService() {
  if (!playing) return;
  if ((long)(millis() - playNext) < 0) return;
  if (playIdx >= playLen) { playing = false; applyVisible(0); return; }
  applyVisible(playBuf[playIdx++]);          // bit i -> LED i (0..7)
  playNext = millis() + playMs;
}

// =============================================================================
//  Commands
// =============================================================================
void cmdVerifyDark() {                       // 'G'
  applyVisible(0);
  if (!waitMs(cfg.ledSettleMs)) { lastErr = ERR_ABORTED; return; }
  if (!measureFrame()) { lastErr = ERR_ABORTED; return; }
  bool alert = false;
  for (uint8_t i = 0; i < NUM_SENS; i++) if (frame[i] > cfg.darkThreshold) alert = true;
  setFlag(F_DARK, alert);
}

void cmdSingleLed(uint8_t n) {               // 'H'
  if (n >= NUM_VISIBLE) { lastErr = ERR_ARG; return; }
  applyVisible(1u << n);
  bool ok = waitMs(cfg.ledSettleMs) && measureFrame();
  applyVisible(0);
  if (!ok) lastErr = ERR_ABORTED;
}

// Sets visible LED i to levels[i] (0..4095). UVC channels are never touched here.
void applyLevels(const uint16_t *levels) {
  uint16_t mask = 0;
  for (uint8_t ch = 0; ch < NUM_VISIBLE; ch++) {
    uint16_t v = min(levels[ch], (uint16_t)4095);
    Tlc.set(ch, v);
    if (v) mask |= (1u << ch);
  }
  visibleMask = mask;
  tlcCommit();
}

void cmdPattern(const uint8_t *b, uint8_t n) {  // 'X'
  if (n != 1 + 2 * NUM_VISIBLE) { lastErr = ERR_ARG; return; }
  uint16_t levels[NUM_VISIBLE];
  for (uint8_t i = 0; i < NUM_VISIBLE; i++) {
    levels[i] = b[1 + 2 * i] | (b[2 + 2 * i] << 8);
    if (levels[i] > 4095) { lastErr = ERR_ARG; return; }
  }
  applyLevels(levels);
  bool ok = waitMs(cfg.ledSettleMs) && measureFrameN(PATTERN_SAMPLES);
  applyVisible(0);
  if (!ok) lastErr = ERR_ABORTED;
}

void cmdByte(uint8_t value) {                // 'K'
  applyVisible(value);                       // LEDs 0..7
  if (!waitMs(cfg.ledSettleMs) || !measureFrame()) lastErr = ERR_ABORTED;
}

void cmdCalibrate() {                        // 'C'
  setFlag(F_DATA, false);
  for (uint8_t row = 0; row <= NUM_VISIBLE; row++) {
    applyVisible(row == 0 ? 0 : (1u << (row - 1)));
    if (!waitMs(cfg.ledSettleMs) || !readAll(calib[row])) {
      applyVisible(0);
      lastErr = ERR_ABORTED;
      return;
    }
  }
  applyVisible(0);
  memcpy(frame, calib[0], sizeof(frame));
  setFlag(F_DATA, true);
}

void cmdSafe() {                             // 'D'
  playing = false;
  uvKill();
  applyVisible(0);
}

void cmdParam(uint8_t id, uint16_t v) {      // 'P'
  switch (id) {
    case 0: cfg.ledSettleMs   = min(v, (uint16_t)5000); break;
    case 1: cfg.darkThreshold = min(v, (uint16_t)1023); break;
    case 2: cfg.visLevel      = min(v, (uint16_t)4095); applyVisible(visibleMask); break;
    case 3: cfg.uvLevel       = min(v, (uint16_t)4095); if (uvLit) applyUV(true); break;
    case 4: cfg.samples       = constrain(v, 1, 16); break;
    case 5: cfg.uvDefaultSec  = constrain(v, 1, UV_HARD_MAX_SEC); break;
    default: lastErr = ERR_ARG; break;
  }
}

void execute(const uint8_t *b, uint8_t n) {
  uint8_t c = b[0];
  lastCmd = c;
  lastErr = ERR_NONE;

  // Emergencies first
  if (c == 'D') { abortReq = false; cmdSafe(); return; }
  if (c == 'F') { abortReq = false; uvStop(); return; }

  // While UVC is active only 'F', 'D', 'L' and 'P' are accepted
  if (uvPhase != UV_OFF && c != 'L' && c != 'P') { lastErr = ERR_UV_ACTIVE; return; }

  if (c != 'B' && c != 'L' && c != 'P') playing = false;

  state = ST_BUSY;
  switch (c) {
    case 'G': cmdVerifyDark(); break;
    case 'H': if (n >= 2) cmdSingleLed(b[1]); else lastErr = ERR_ARG; break;
    case 'K': if (n >= 2) cmdByte(b[1]);      else lastErr = ERR_ARG; break;
    case 'M': if (n >= 3) applyVisible(b[1] | (b[2] << 8)); else lastErr = ERR_ARG; break;
    case 'X': cmdPattern(b, n); break;
    case 'S': if (!measureFrame()) lastErr = ERR_ABORTED; break;
    case 'O': applyVisible(0); break;
    case 'C': cmdCalibrate(); break;
    case 'L': if (n >= 2 && b[1] <= 1) setRange(b[1]); else lastErr = ERR_ARG; break;
    case 'P':
      if (n >= 4) cmdParam(b[1], b[2] | (b[3] << 8)); else lastErr = ERR_ARG;
      break;
    case 'B': {
      if (n < 4) { lastErr = ERR_ARG; break; }
      playMs  = max((uint16_t)(b[1] | (b[2] << 8)), (uint16_t)1);
      playLen = min((uint8_t)(n - 3), (uint8_t)sizeof(playBuf));
      memcpy(playBuf, b + 3, playLen);
      playIdx = 0;
      playNext = millis();
      playing = true;
      break;
    }
    case 'E': {
      uint16_t sec = (n >= 3) ? (b[1] | (b[2] << 8)) : cfg.uvDefaultSec;
      if (sec == 0 || sec > UV_HARD_MAX_SEC) { lastErr = ERR_ARG; break; }
      uvStart(sec);
      break;
    }
    default: lastErr = ERR_UNKNOWN; break;
  }
  if (abortReq && lastErr == ERR_NONE) lastErr = ERR_ABORTED;
}

void updateState() {
  if (uvPhase != UV_OFF)  state = ST_UV;
  else if (playing)       state = ST_PLAY;
  else                    state = ST_IDLE;
}

void updateFlags() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();
  setFlag(F_XERR, analogRead(PIN_TLC_XERR) < 512);
  setFlag(F_INT,  digitalRead(PIN_INT_EXP) == LOW);
}

// =============================================================================
//  I2C (interrupt context: copy data only, nothing slow)
// =============================================================================
void preparePage(uint8_t page) {
  const uint16_t *src = nullptr;
  uint16_t tmp[NUM_SENS];
  if (page == 0) src = frame;
  else if (page <= NUM_VISIBLE + 1) src = calib[page - 1];
  else if (page == 14) {
    memset(tmp, 0, sizeof(tmp));
    tmp[0] = cfg.ledSettleMs; tmp[1] = cfg.darkThreshold; tmp[2] = cfg.visLevel;
    tmp[3] = cfg.uvLevel;     tmp[4] = cfg.samples;       tmp[5] = cfg.uvDefaultSec;
    tmp[6] = UV_HARD_MAX_SEC; tmp[7] = FW_VERSION;
    src = tmp;
  }
  for (uint8_t i = 0; i < NUM_SENS; i++) {
    uint16_t v = src ? src[i] : 0xFFFF;      // invalid page -> 0xFFFF
    txData[2 * i]     = v & 0xFF;
    txData[2 * i + 1] = v >> 8;
  }
}

void onI2CReceive(int count) {
  if (count <= 0) return;
  uint8_t c = Wire.read();
  if (c == '?') {
    respMode = RESP_STATUS;
  } else if (c == 'R') {
    uint8_t page = Wire.available() ? Wire.read() : 0;
    preparePage(page);
    respMode = RESP_DATA;
  } else {
    bool emergency = (c == 'F' || c == 'D');
    respMode = RESP_STATUS;
    if (cmdPending && !emergency) {
      lastErr = ERR_BUSY;
    } else {
      if (emergency) abortReq = true;        // interrupts whatever is running
      rxBuf[0] = c;
      uint8_t len = 1;
      while (Wire.available() && len < sizeof(rxBuf)) rxBuf[len++] = Wire.read();
      rxLen = len;
      cmdPending = true;
      if (state == ST_IDLE) state = ST_BUSY; // the master sees BUSY right away
    }
  }
  while (Wire.available()) Wire.read();      // discard leftovers
}

void onI2CRequest() {
  if (respMode == RESP_DATA) {
    Wire.write((const uint8_t *)txData, 32);
    return;
  }
  uint8_t s[8];
  uint16_t r = uvRemaining;
  s[0] = state;   s[1] = flags;   s[2] = lastCmd; s[3] = lastErr;
  s[4] = FW_VERSION;
  s[5] = r & 0xFF; s[6] = r >> 8;
  s[7] = i2cAddr;
  Wire.write(s, 8);
}

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();

  // Safe outputs before anything else
  pinMode(PIN_UV_EN, OUTPUT);    digitalWrite(PIN_UV_EN, LOW);
  pinMode(PIN_TLC_MODE, OUTPUT); digitalWrite(PIN_TLC_MODE, LOW);  // grayscale mode
  for (uint8_t b = 0; b < 4; b++) { pinMode(PIN_MUX_S[b], OUTPUT); digitalWrite(PIN_MUX_S[b], LOW); }
  pinMode(PIN_INT_EXP, INPUT_PULLUP);
  setRange(false);

  Tlc.init(0);                   // all channels at 0 (UVC included)
  tlcCommit();

  if (mcusr & (1 << WDRF)) setFlag(F_WDT, true);

  // I2C address from the solder jumpers (closed = 1)
  pinMode(PIN_ADDR0, INPUT_PULLUP);
  pinMode(PIN_ADDR1, INPUT_PULLUP);
  delay(2);
  uint8_t a = (digitalRead(PIN_ADDR0) == LOW ? 1 : 0) |
              ((digitalRead(PIN_ADDR1) == LOW ? 1 : 0) << 1);
  i2cAddr = 0x10 + a;

#if DEBUG
  Serial.begin(115200);
  DBG(F("WALCORG node v4.5 fw ")); DBG(FW_VERSION); DBG(F(" addr 0x")); Serial.println(i2cAddr, HEX);
#endif

  Wire.begin(i2cAddr);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

#if USE_WATCHDOG
  wdt_enable(WDTO_2S);           // if something hangs: reset -> UVC off
#endif
}

void loop() {
  petDog();

  if (cmdPending) {
    uint8_t buf[32], len;
    noInterrupts();
    len = rxLen;
    for (uint8_t i = 0; i < len; i++) buf[i] = rxBuf[i];
    cmdPending = false;
    interrupts();
    execute(buf, len);
  }

  uvService();
  playService();
  updateFlags();
  if (!cmdPending) updateState();
}
