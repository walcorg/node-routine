/*
  ============================================================================
  WALCORG TERRAFORMING CORP. - Nodo v4.5 (modelo de adquisicion de datos)
  Firmware unificado para Arduino Nano (ATmega328P)          Version 0.1
  ============================================================================

  Une en un solo sketch las rutinas anteriores:
    - rutine_verify_photo_zero_v1/v2   -> comando 'G' (verificar oscuridad)
    - rutine_photo_sampling_v1/v2      -> comando 'H' (un LED + lectura)
    - password_sha-256 / leds_codif.   -> comando 'K' (byte en 8 LEDs + lectura)
    - rutine_modules_control_v1 (B,C,D)-> comandos 'B', 'C', 'D'
    - rutine_sterelization_uv (E,F)    -> comandos 'E', 'F'
  Todo por I2C (esclavo); ya no se usa el puerto serie para control.

  ---------------------------------------------------------------------------
  HARDWARE (PCB rigida v4.5)
  ---------------------------------------------------------------------------
    TLC59401 (16 canales, sumidero de corriente):
      D3  GSCLK   D9  XLAT   D10 BLANK   D11 SIN   D13 SCLK  (los usa la libreria)
      D12 MODE  (VPRG: LOW = modo escala de grises)
      A6  XERR  (drenador abierto, pull-up 10k; LOW = LED abierto o sobretemperatura)
      Canales:  0-7  LED_K0..K7   LEDs de datos (1 bit cada uno, LSB = LED_K0)
                8-11 LED_K8..K11  LEDs de calibracion
                12-15 UV_K12..K15 LEDs UVC (anodo en +V_UV, 9 V)
    CD74HC4067 (multiplexor analogico 16:1):
      D4-D7 S0-S3   A0 MUX_SIG (COMMON)   E a GND
      Entradas: 0-7 SENS_B0..B7 (retrodispersion, J1)
                8-15 SENS_T0..T7 (transmision, J2)
    A3  RANGE_SEL  salida LOW = rango normal (~9.1k)
                   entrada (alta Z) = rango sensible (100k). NUNCA salida HIGH.
    A2  UV_EN      habilita el elevador MT3608 (+V_UV). LOW = apagado.
    A1  INT_EXP1   interrupcion de modulos de expansion (activa en LOW)
    D2  ADDR0, D8 ADDR1  solder jumpers a GND -> direccion 0x10..0x13
    A4/A5 SDA/SCL  bus I2C hacia el shield (pull-ups 4.7k en el shield)

  Libreria requerida: "Tlc5940" (Alex Leone; fork de Paul Stoffregen).
  Con su configuracion por defecto (NUM_TLCS 1, VPRG_ENABLED 0, TLC_SPI) los
  pines coinciden con esta placa. Usa Timer1 y Timer2: no usar analogWrite()
  en D3, D9, D10, D11 ni tone().

  ---------------------------------------------------------------------------
  PROTOCOLO I2C  (direccion 0x10 + ADDR1*2 + ADDR0; jumper cerrado = 1)
  ---------------------------------------------------------------------------
  El maestro escribe [comando, argumentos...] (maximo 32 bytes).
  Despues de cualquier comando (excepto 'R'), una lectura devuelve el ESTADO:
     8 bytes: [estado, banderas, ultimo_cmd, error, version_fw,
               uv_seg_restantes_L, uv_seg_restantes_H, direccion_i2c]
  'R',pagina prepara una pagina de DATOS; la siguiente lectura devuelve
     32 bytes = 16 valores uint16 little-endian (0-1023):
       pagina 0      ultima medicion (16 sensores, indice = canal del mux)
       pagina 1      calibracion: oscuridad (todos los LEDs apagados)
       pagina 2..13  calibracion: LED 0..11 encendido solo
       pagina 14     configuracion actual (parametros 0..5, ver 'P')
  '?' vuelve a modo ESTADO sin ejecutar nada.

  Comandos:
   'G'                    Verifica oscuridad: apaga LEDs y mide los 16 sensores.
                          Activa la bandera DARK_ALERT si alguno > umbral.
   'H', n                 Enciende solo el LED n (0-11), mide y lo apaga.
   'K', byte              Muestra el byte en los LEDs 0-7 (bit i -> LED i),
                          espera, mide y deja el patron encendido.
                          Es la primitiva para codificar texto caracter a caracter.
   'M', maskL, maskH      Enciende los LEDs visibles indicados por la mascara (12 bits).
   'S'                    Mide los 16 sensores con los LEDs como esten.
   'O'                    Apaga todos los LEDs visibles.
   'B', vL, vH, texto...  Reproduce el texto en los LEDs 0-7, v = ms por caracter
                          (sin medir; para demostracion). Hasta 29 caracteres.
   'C'                    Calibracion completa: oscuridad + cada LED 0-11 solo.
                          Resultados en las paginas 1-13.
   'E' [, sL, sH]         Desinfeccion UVC durante s segundos (defecto 300,
                          maximo 1800). Parpadeo rapido al encender y al apagar.
   'F'                    Detiene la desinfeccion UVC.
   'D'                    Modo seguro: apaga TODO de inmediato (LEDs, UVC, elevador).
   'L', r                 Rango de medicion: 0 normal, 1 sensible.
   'P', id, vL, vH        Ajusta un parametro:
                            0 tiempo de estabilizacion de LEDs (ms, defecto 30)
                            1 umbral de oscuridad (0-1023, defecto 20)
                            2 intensidad LEDs visibles (0-4095, defecto 4095)
                            3 intensidad UVC (0-4095, defecto 4095)
                            4 muestras promediadas por sensor (1-16, defecto 4)
                            5 duracion UVC por defecto (s, defecto 300)
  'F' y 'D' se atienden aunque haya otra operacion en curso.

  Estados: 0 LIBRE, 1 OCUPADO, 2 UVC (encendiendo/encendido/apagando), 3 REPRODUCIENDO
  Banderas (bits): 0 DATOS_LISTOS  1 DARK_ALERT  2 XERR  3 UV_ACTIVO
                   4 RANGO_SENSIBLE  5 INT_EXP  6 REINICIO_POR_WATCHDOG
  Errores: 0 ninguno, 1 comando desconocido, 2 argumento invalido,
           3 ocupado (ya habia un comando pendiente), 4 bloqueado por UVC activo,
           5 abortado por 'F' o 'D'
  ============================================================================
*/

#include <Wire.h>
#include <Tlc5940.h>
#include <avr/wdt.h>
#include <string.h>

// ----------------------------- Opciones de compilacion ----------------------
#define FW_VERSION   0x01
#define USE_WATCHDOG 1   // Poner en 0 si el Nano tiene el bootloader antiguo
                         // (con ese bootloader un reinicio por watchdog lo deja
                         //  en un ciclo de reinicios).
#define DEBUG        0   // 1 = mensajes por Serial a 115200 (D0/D1 libres)

#if DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// ----------------------------- Pines -----------------------------------------
const uint8_t PIN_ADDR0     = 2;
const uint8_t PIN_ADDR1     = 8;
const uint8_t PIN_MUX_S[4]  = {4, 5, 6, 7};
const uint8_t PIN_TLC_MODE  = 12;
const uint8_t PIN_MUX_SIG   = A0;
const uint8_t PIN_INT_EXP   = A1;
const uint8_t PIN_UV_EN     = A2;
const uint8_t PIN_RANGE_SEL = A3;
const uint8_t PIN_TLC_XERR  = A6;   // solo analogica

// ----------------------------- Mapa de canales ------------------------------
const uint8_t CH_DATA0     = 0;    // LEDs de datos 0..7
const uint8_t NUM_VISIBLE  = 12;   // canales 0..11 (datos + calibracion)
const uint8_t CH_UV0       = 12;   // UVC 12..15
const uint8_t NUM_UV       = 4;
const uint8_t NUM_SENS     = 16;

// ----------------------------- Limites de seguridad -------------------------
const uint16_t UV_HARD_MAX_SEC = 1800;  // 30 min, no se puede superar por I2C
const uint16_t UV_BLINK_MS     = 30;
const uint8_t  UV_BLINK_COUNT  = 10;
const uint16_t UV_BOOST_START_MS = 10;  // arranque del MT3608 antes del 1er parpadeo

// ----------------------------- Codigos ---------------------------------------
enum : uint8_t { ST_IDLE = 0, ST_BUSY = 1, ST_UV = 2, ST_PLAY = 3 };
enum : uint8_t { ERR_NONE = 0, ERR_UNKNOWN = 1, ERR_ARG = 2, ERR_BUSY = 3,
                 ERR_UV_ACTIVE = 4, ERR_ABORTED = 5 };
enum : uint8_t { F_DATA = 0, F_DARK = 1, F_XERR = 2, F_UV = 3,
                 F_RANGE = 4, F_INT = 5, F_WDT = 6 };
enum : uint8_t { RESP_STATUS = 0, RESP_DATA = 1 };
enum : uint8_t { UV_OFF = 0, UV_RAMP_ON, UV_ON, UV_RAMP_OFF };

// ----------------------------- Configuracion --------------------------------
struct Config {
  uint16_t ledSettleMs   = 30;    // fotorresistencias/fototransistores
  uint16_t darkThreshold = 20;
  uint16_t visLevel      = 4095;
  uint16_t uvLevel       = 4095;
  uint16_t samples       = 4;
  uint16_t uvDefaultSec  = 300;
} cfg;
const uint8_t NUM_PARAMS = 6;

// ----------------------------- Datos -----------------------------------------
uint16_t frame[NUM_SENS];                     // ultima medicion
uint16_t calib[1 + NUM_VISIBLE][NUM_SENS];    // fila 0 oscuridad, 1..12 LEDs
uint16_t visibleMask = 0;                     // LEDs visibles encendidos
bool     rangeSensitive = false;

// ----------------------------- Compartido con la ISR de I2C -----------------
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
unsigned long uvT0 = 0;          // inicio de la fase actual / ultimo parpadeo
unsigned long uvDurationMs = 0;

// ----------------------------- Reproduccion de texto ------------------------
bool          playing = false;
uint8_t       playBuf[29];
uint8_t       playLen = 0, playIdx = 0;
uint16_t      playMs = 100;
unsigned long playNext = 0;

// =============================================================================
//  Utilidades
// =============================================================================
static inline void setFlag(uint8_t bit, bool on) {
  uint8_t f = flags;
  if (on) f |= (1 << bit); else f &= ~(1 << bit);
  flags = f;                        // escritura de un byte: atomica
}

static inline void petDog() {
#if USE_WATCHDOG
  wdt_reset();
#endif
}

// Espera ms atendiendo watchdog y aborto. Devuelve false si se aborto.
bool waitMs(uint16_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    petDog();
    if (abortReq) return false;
  }
  return true;
}

// Escribe los datos al TLC (espera si el envio anterior no se ha latcheado).
void tlcCommit() {
  while (Tlc.update()) { petDog(); }
}

void applyVisible(uint16_t mask) {
  mask &= 0x0FFF;                   // solo canales 0..11; los UVC nunca se tocan aqui
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
    pinMode(PIN_RANGE_SEL, INPUT);          // alta impedancia -> solo 100k
  } else {
    digitalWrite(PIN_RANGE_SEL, LOW);       // primero LOW para no pasar por HIGH
    pinMode(PIN_RANGE_SEL, OUTPUT);         // 10k en paralelo -> ~9.1k
  }
  rangeSensitive = sensitive;
  setFlag(F_RANGE, sensitive);
}

void setMux(uint8_t ch) {
  for (uint8_t b = 0; b < 4; b++) digitalWrite(PIN_MUX_S[b], (ch >> b) & 1);
}

// Mide los 16 sensores. Descarta la primera conversion de cada canal porque
// el capacitor de muestreo del ADC tarda en cargarse con fuentes de alta Z.
bool readAll(uint16_t *out) {
  uint8_t n = constrain(cfg.samples, 1, 16);
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

// Mide hacia frame[] y marca datos listos.
bool measureFrame() {
  setFlag(F_DATA, false);
  if (!readAll(frame)) return false;
  setFlag(F_DATA, true);
  return true;
}

// =============================================================================
//  UVC: maquina de estados no bloqueante
// =============================================================================
void uvStart(uint16_t sec) {
  playing = false;
  applyVisible(0);                          // presupuesto de corriente del USB
  digitalWrite(PIN_UV_EN, HIGH);            // arranca el elevador
  uvDurationMs = (unsigned long)sec * 1000UL;
  uvPhase = UV_RAMP_ON;
  uvToggles = 0;
  uvT0 = millis() + UV_BOOST_START_MS;      // primer parpadeo tras el arranque
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

void uvKill() {                              // apagado inmediato
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
          applyUV(true);                    // queda encendido fijo
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
//  Reproduccion de texto en los 8 LEDs de datos
// =============================================================================
void playService() {
  if (!playing) return;
  if ((long)(millis() - playNext) < 0) return;
  if (playIdx >= playLen) { playing = false; applyVisible(0); return; }
  applyVisible(playBuf[playIdx++]);          // bit i -> LED i (0..7)
  playNext = millis() + playMs;
}

// =============================================================================
//  Comandos
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

  // Emergencias primero
  if (c == 'D') { abortReq = false; cmdSafe(); return; }
  if (c == 'F') { abortReq = false; uvStop(); return; }

  // Con UVC activo solo se aceptan 'F', 'D', 'L' y 'P'
  if (uvPhase != UV_OFF && c != 'L' && c != 'P') { lastErr = ERR_UV_ACTIVE; return; }

  if (c != 'B' && c != 'L' && c != 'P') playing = false;

  state = ST_BUSY;
  switch (c) {
    case 'G': cmdVerifyDark(); break;
    case 'H': if (n >= 2) cmdSingleLed(b[1]); else lastErr = ERR_ARG; break;
    case 'K': if (n >= 2) cmdByte(b[1]);      else lastErr = ERR_ARG; break;
    case 'M': if (n >= 3) applyVisible(b[1] | (b[2] << 8)); else lastErr = ERR_ARG; break;
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
//  I2C (contexto de interrupcion: solo copiar datos, nada lento)
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
    uint16_t v = src ? src[i] : 0xFFFF;      // pagina invalida -> 0xFFFF
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
      if (emergency) abortReq = true;        // interrumpe lo que este en curso
      rxBuf[0] = c;
      uint8_t len = 1;
      while (Wire.available() && len < sizeof(rxBuf)) rxBuf[len++] = Wire.read();
      rxLen = len;
      cmdPending = true;
      if (state == ST_IDLE) state = ST_BUSY; // el maestro vera OCUPADO de inmediato
    }
  }
  while (Wire.available()) Wire.read();      // descartar sobrantes
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

  // Salidas seguras antes que nada
  pinMode(PIN_UV_EN, OUTPUT);    digitalWrite(PIN_UV_EN, LOW);
  pinMode(PIN_TLC_MODE, OUTPUT); digitalWrite(PIN_TLC_MODE, LOW);  // escala de grises
  for (uint8_t b = 0; b < 4; b++) { pinMode(PIN_MUX_S[b], OUTPUT); digitalWrite(PIN_MUX_S[b], LOW); }
  pinMode(PIN_INT_EXP, INPUT_PULLUP);
  setRange(false);

  Tlc.init(0);                   // todos los canales en 0 (incluidos los UVC)
  tlcCommit();

  if (mcusr & (1 << WDRF)) setFlag(F_WDT, true);

  // Direccion I2C desde los solder jumpers (cerrado = 1)
  pinMode(PIN_ADDR0, INPUT_PULLUP);
  pinMode(PIN_ADDR1, INPUT_PULLUP);
  delay(2);
  uint8_t a = (digitalRead(PIN_ADDR0) == LOW ? 1 : 0) |
              ((digitalRead(PIN_ADDR1) == LOW ? 1 : 0) << 1);
  i2cAddr = 0x10 + a;

#if DEBUG
  Serial.begin(115200);
  DBG(F("WALCORG nodo v4.5 fw ")); DBG(FW_VERSION); DBG(F(" addr 0x")); Serial.println(i2cAddr, HEX);
#endif

  Wire.begin(i2cAddr);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

#if USE_WATCHDOG
  wdt_enable(WDTO_2S);           // si algo se cuelga: reinicio -> UVC apagado
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
