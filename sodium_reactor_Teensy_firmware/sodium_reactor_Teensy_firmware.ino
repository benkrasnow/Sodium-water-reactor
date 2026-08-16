/*
  Sodium-Water Reactor Test Stand -- Teensy 3.2 firmware
  ------------------------------------------------------
  Collects junction current/voltage/force, linear-actuator position, and VESC
  telemetry, streams it to a host PC over USB serial at ~9 Hz, and accepts
  actuator commands from the host (single-page web app, Chrome Web Serial).

  Protocol (ASCII, one record per line, human readable).  Every line is:

      $<payload>*<CS>\n          CS = 8-bit XOR of every character of <payload>,
                                 printed as 2 uppercase hex digits.

  Teensy -> Host
      $D  ...   telemetry sample  (~9 Hz, space separated key=value)
      $K  ...   configuration / setpoint dump (sent after any SET, and on GET)
      $A  ...   acknowledgement
      $E  ...   error
      $I  ...   informational / boot banner

  Host -> Teensy   (the trailing *CS is validated when present, optional)
      $SET k=v k=v ...   set one or more parameters (see applyParam() below)
      $GET               dump configuration ($K)
      $TARE [n=10]       zero the force sensor, averaging n conversions
      $CAL w=<grams> [n=10]   solve for HX711 counts/gram using a known mass
      $ZERO [n=20]       capture the present current-sensor output as 0 A
      $ADCDIAG           report ADC mux settling error on each analog channel
      $STOP              emergency stop: idle mode, all PWM 0, VESC released
      $SAVE / $LOAD / $DEFAULTS   EEPROM calibration handling
      $PING              host heartbeat (also feeds the comms watchdog)
*/

//Hardware pin connections
#define PIN_ADC_CURRENT_SENSE 23  //Clamp-on current sensor for junction current.  Hass 50-S.  2.5V baseline.  Value changes +/- .625V for each 50 amps
#define PIN_ADC_VOLTAGE_SENSE 22  //Direct reading of junction voltage

#define PIN_ADC_POSITION_SENSE 21  //Analog voltage from optical distance sensor

#define PIN_SERIAL_ESC_TX 1  // Serial1 is the hardware serial port for ESC control
#define PIN_SERIAL_ESC_RX 0

#define PIN_LINEAR_FWD 5  //Bi-polar L6203 motor driver.  _EN should be high at all times.  PWM should be supplied to either _FWD or _REV
#define PIN_LINEAR_REV 6
#define PIN_LINEAR_EN 8

#define PIN_WATER_PUMP_SUPPLY 3  //Bi-polar L6203 motor driver.  _EN should be high at all times.  PWM should be supplied to  _SUPPLY
#define PIN_WATER_PUMP_GND 4  //This is a bipolar motor driver, but the water pump is meant to spin one way.  This pin should always be low.
#define PIN_WATER_PUMP_EN 7

#define PIN_HX711_DOUT 10  //HX711 interface pins
#define PIN_HX711_SCK 11



#define MOTOR_PWM_BITS 10
#define MOTOR_PWM_RANGE ( 1 << MOTOR_PWM_BITS)
#define MOTOR_PWM_MAX (MOTOR_PWM_RANGE - 1)
#define MOTOR_PWM_FREQUENCY 18000

#define ADC_BITS 16
#define ADC_COUNTS 65535.0f
#define ADC_VREF 3.3f

#define CYCLE_MS 110          // main acquisition/control period -> ~9.09 Hz
                              // (HX711 delivers 10 samples/s, so this is the
                              //  fastest the force loop can honestly run)

#include <VescUart.h>
VescUart VESC;

#define FORCE_CALIBRATION_FACTOR 230.0f   // HX711 counts per gram (default)

#include <EEPROM.h>


// ---------------------------------------------------------------- modes ----
#define MODE_IDLE   0   // hold position, no drive
#define MODE_FORCE  1   // closed-loop force control (PID on HX711)
#define MODE_RESET  2   // fixed signed PWM until distance target reached
#define MODE_MANUAL 3   // host commands linear PWM directly

// ------------------------------------------------------------ status flags --
#define FLG_VESC_TIMEOUT  0x01
#define FLG_RESET_TIMEOUT 0x02
#define FLG_WATCHDOG      0x04
#define FLG_FORCE_BAD     0x08   // no fresh HX711 conversion
#define FLG_PID_SAT       0x10
#define FLG_FORCE_SAT     0x20   // load cell / HX711 input railed
#define FLG_CALIBRATING   0x40   // a tare or scale average is in progress


// =====================================================================
//  HX711 driver -- bit-banged, non-blocking, one conversion per cycle
//
//  The HX711 pulls DOUT low when a conversion is ready and holds it there
//  until the sample is clocked out.  The acquisition cycle (110 ms) is
//  deliberately slower than the converter (10 SPS / 100 ms), so a fresh
//  sample is always already waiting when we look: we never wait, never time
//  out, and never need a timeout policy that can silently return zero.
//
//  Clocking out one sample takes ~60 us.  PD_SCK must never stay high for
//  more than 50 us or the chip begins its power-down sequence, so the burst
//  runs with interrupts disabled.
// =====================================================================
#define HX_GAIN_128  1   // channel A, gain 128 -> 25 clock pulses
#define HX_GAIN_32   2   // channel B, gain  32 -> 26 clock pulses
#define HX_GAIN_64   3   // channel A, gain  64 -> 27 clock pulses

#define HX_GAIN      HX_GAIN_128
#define HX_STALE_MS  400          // 4 missed conversions -> sensor is not talking
#define HX_SAT_COUNT 8388000L     // 24-bit full scale is +/-8388607

static long     hxRaw     = 0;    // most recent raw conversion, sign extended
static uint32_t hxLastMs  = 0;
static bool     hxHave    = false;
static uint8_t  hxDiscard = 3;    // the HX711 needs ~400ms to settle after power-up

static void hxBegin() {
  pinMode(PIN_HX711_SCK, OUTPUT);
  pinMode(PIN_HX711_DOUT, INPUT);
  digitalWrite(PIN_HX711_SCK, LOW);   // SCK low keeps the converter awake
}

static inline bool hxReady() { return digitalRead(PIN_HX711_DOUT) == LOW; }

// Clock out one conversion.  Only call when hxReady() is true.
static long hxReadOnce() {
  uint32_t v = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_HX711_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  for (uint8_t i = 0; i < HX_GAIN; i++) {   // extra pulses select gain/channel for the NEXT conversion
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  interrupts();
  if (v & 0x800000UL) v |= 0xFF000000UL;    // sign extend 24 -> 32 bits
  return (long)(int32_t)v;
}

// Take whatever is pending and keep the newest.  Returns true if a new
// conversion was captured.  Never blocks.
static bool hxUpdate() {
  bool got = false;
  for (uint8_t i = 0; i < 4 && hxReady(); i++) {
    long v = hxReadOnce();
    hxLastMs = millis();
    delayMicroseconds(2);          // let DOUT settle high before re-testing
    if (hxDiscard) { hxDiscard--; continue; }   // throw away the settling conversions
    hxRaw = v;
    hxHave = true;
    got = true;
  }
  return got;
}


// ------------------------------------------------ averaging jobs (tare/cal) --
// Both tare and scale calibration need an average of N conversions.  Rather
// than blocking for N*100 ms, the samples are accumulated by the normal
// acquisition cycle and the job completes on its own N cycles later.
#define JOB_NONE 0
#define JOB_TARE 1
#define JOB_CAL  2

static uint8_t  jobKind      = JOB_NONE;
static uint16_t jobRemaining = 0;
static uint16_t jobCount     = 0;
static int64_t  jobAccum     = 0;
static float    jobGrams     = 0.0f;   // known mass, JOB_CAL only

// The junction-current zero is averaged the same way: over whole acquisition
// cycles, so that it sees exactly what the run loop sees.
static uint16_t izRemaining  = 0;
static uint16_t izCount      = 0;
static int64_t  izAccum      = 0;

static void startJob(uint8_t kind, uint16_t n, float grams);   // defined with the acquisition code


// -------------------------------------------------------------- config -----
#define CFG_MAGIC 0x53575232UL   // 'SWR2'

struct Config {
  uint32_t magic;

  // junction current sensor (Hass 50-S: 2.5 V at 0 A, 0.625 V per 50 A)
  float i_zero_v;      // ADC volts that correspond to 0 A
  float i_v_per_a;     // volts per amp (0.625/50 = 0.0125)

  // junction voltage front end (V at the junction per V at the ADC pin)
  float v_gain;
  float v_off;

  // optical distance sensor: mm = d_gain * volts + d_off
  float d_gain;        // 28.2 mm/V
  float d_off;         // -35 mm

  // force
  float f_cal;         // HX711 scale factor (counts per gram)
  long  f_offset;      // HX711 raw counts at zero load (tare)
  float f_alpha;       // EMA smoothing 0..1  (1.0 = no filtering)

  // force PID
  float kp, ki, kd;
  float pid_out_max;   // |PWM| clamp
  float pid_min_pwm;   // stiction floor: non-zero outputs are pushed to at least this
  float pid_db;        // deadband, grams
  int16_t f_sign;      // +1 if LINEAR_FWD increases force, -1 otherwise

  // reset (retract) mode
  float reset_pwm;     // signed PWM applied while retracting; the sign is the
                       // retract direction (negative drives LINEAR_REV)
  float reset_target;  // distance target, same units as dmm
  int16_t reset_cmp;   // 0: finish when distance >= target,  1: when distance <= target
  uint32_t reset_timeout_ms;

  // VESC
  int16_t poles;       // motor pole pairs (erpm = rpm * poles)
  int16_t vesc_poll;   // 0 = do not talk to the VESC at all

  uint32_t wd_ms;      // host comms watchdog, 0 = disabled
};

Config cfg;

void loadDefaults() {
  cfg.magic            = CFG_MAGIC;
  cfg.i_zero_v         = 2.500f;
  cfg.i_v_per_a        = 0.0125f;
  cfg.v_gain           = 1.0f;
  cfg.v_off            = 0.0f;
  cfg.d_gain           = 28.2f;    // mm = volts * 28.2 - 35
  cfg.d_off            = -35.0f;
  cfg.f_cal            = FORCE_CALIBRATION_FACTOR;
  cfg.f_offset         = 0;
  cfg.f_alpha          = 0.7f;
  cfg.kp               = 0.50f;
  cfg.ki               = 1.00f;
  cfg.kd               = 0.00f;
  cfg.pid_out_max      = 700.0f;
  cfg.pid_min_pwm      = 0.0f;
  cfg.pid_db           = 2.0f;
  cfg.f_sign           = 1;
  cfg.reset_pwm        = -1000.0f;   // negative retracts on this machine
  cfg.reset_target     = 0.0f;
  cfg.reset_cmp        = 0;
  cfg.reset_timeout_ms = 20000;
  cfg.poles            = 7;
  cfg.vesc_poll        = 1;
  cfg.wd_ms            = 2500;
}


// ------------------------------------------------------------ live state ---
uint32_t seq          = 0;
uint32_t lastCycleMs  = 0;
uint32_t lastHostMs   = 0;
bool     hostSeen     = false;

int      linMode      = MODE_IDLE;
float    forceSetpoint = 0.0f;       // grams
int      manualPWM    = 0;           // -MAX..MAX, used in MODE_MANUAL
int      linearPWM    = 0;           // what is actually being driven, signed
int      pumpPWM      = 0;           // 0..MAX
float    rpmSetpoint  = 0.0f;        // mechanical RPM requested of the VESC

uint16_t flags        = 0;

// sensor values
uint16_t rawCurrent = 0, rawVoltage = 0, rawDistance = 0;
float junctionCurrent = 0.0f;   // A
float junctionVoltage = 0.0f;   // V
float junctionPower   = 0.0f;   // W
float distanceVolts   = 0.0f;
float distanceMM      = 0.0f;
float forceG          = 0.0f;   // filtered
bool  forceValid      = false;
bool  forceFiltPrimed = false;

// VESC telemetry
bool  vescOK = false;
float vescERPM = 0.0f, vescRPM = 0.0f;
float vescMotorCurrent = 0.0f, vescInputCurrent = 0.0f, vescInputVoltage = 0.0f;

// PID state
float pidIntegral = 0.0f;
float pidPrevMeas = 0.0f;
bool  pidPrimed   = false;
float pidError    = 0.0f;
float pidOutput   = 0.0f;

// reset-mode state
uint32_t resetStartMs = 0;


// =====================================================================
//  Transmit helpers -- a line is assembled in txbuf, then checksummed
// =====================================================================
static char     txbuf[640];
static uint16_t txlen = 0;

static void txReset() { txlen = 0; txbuf[0] = 0; }

static void txStr(const char *s) {
  while (*s && txlen < sizeof(txbuf) - 1) txbuf[txlen++] = *s++;
  txbuf[txlen] = 0;
}

static void txInt(long v) {
  char t[16];
  snprintf(t, sizeof(t), "%ld", v);   // integer formatting only -- always available
  txStr(t);
}

static void txFloat(float v, int prec) {
  char t[24];
  if (isnan(v) || isinf(v)) { txStr("nan"); return; }
  dtostrf(v, 0, prec, t);
  // dtostrf can leave a leading space when width is 0 on some cores
  char *p = t;
  while (*p == ' ') p++;
  txStr(p);
}

static void txKeyF(const char *k, float v, int prec) {
  txStr(" "); txStr(k); txStr("="); txFloat(v, prec);
}

static void txKeyI(const char *k, long v) {
  txStr(" "); txStr(k); txStr("="); txInt(v);
}

static void txSend() {
  uint8_t cs = 0;
  for (uint16_t i = 0; i < txlen; i++) cs ^= (uint8_t)txbuf[i];
  char hex[4];
  const char *H = "0123456789ABCDEF";
  hex[0] = H[(cs >> 4) & 0x0F];
  hex[1] = H[cs & 0x0F];
  hex[2] = 0;
  Serial.write('$');
  Serial.write((const uint8_t *)txbuf, txlen);
  Serial.write('*');
  Serial.write((const uint8_t *)hex, 2);
  Serial.write('\n');
}

static void sendSimple(const char *tag, const char *msg) {
  txReset();
  txStr(tag);
  txStr(" ");
  txStr(msg);
  txSend();
}

static void sendAck(const char *msg)   { sendSimple("A", msg); }
static void sendErr(const char *msg)   { sendSimple("E", msg); }
static void sendInfo(const char *msg)  { sendSimple("I", msg); }


// =====================================================================
//  Actuator output
// =====================================================================

// v is signed: positive drives LINEAR_FWD, negative drives LINEAR_REV.
// The opposing channel is always zeroed first so the bridge never sees
// both inputs high.
static void setLinearPWM(int v) {
  if (v >  MOTOR_PWM_MAX) v =  MOTOR_PWM_MAX;
  if (v < -MOTOR_PWM_MAX) v = -MOTOR_PWM_MAX;
  linearPWM = v;
  if (v >= 0) {
    analogWrite(PIN_LINEAR_REV, 0);
    analogWrite(PIN_LINEAR_FWD, v);
  } else {
    analogWrite(PIN_LINEAR_FWD, 0);
    analogWrite(PIN_LINEAR_REV, -v);
  }
}

static void setPumpPWM(int v) {
  if (v < 0) v = 0;
  if (v > MOTOR_PWM_MAX) v = MOTOR_PWM_MAX;
  pumpPWM = v;
  analogWrite(PIN_WATER_PUMP_GND, 0);   // this half of the bridge stays low
  analogWrite(PIN_WATER_PUMP_SUPPLY, v);
}

static void setMode(int m) {
  if (m < MODE_IDLE || m > MODE_MANUAL) return;
  if (m == MODE_FORCE && jobKind != JOB_NONE) {
    sendErr("force mode refused - tare/cal in progress");
    return;
  }
  linMode = m;
  pidIntegral = 0.0f;
  pidPrimed = false;
  flags &= ~(FLG_RESET_TIMEOUT | FLG_PID_SAT);
  if (m == MODE_RESET) resetStartMs = millis();
  if (m == MODE_IDLE)  setLinearPWM(0);
  if (m == MODE_MANUAL) setLinearPWM(manualPWM);
}

static void emergencyStop() {
  manualPWM = 0;
  forceSetpoint = 0.0f;
  rpmSetpoint = 0.0f;
  setMode(MODE_IDLE);
  setLinearPWM(0);
  setPumpPWM(0);
  if (cfg.vesc_poll) VESC.setCurrent(0.0f);
}


// =====================================================================
//  Configuration report
// =====================================================================

// True when what is in EEPROM matches the live config exactly.  Config is a
// global, so its padding bytes are zero, and EEPROM.get overwrites every byte
// of the comparison copy -- memcmp is stable here.
static bool cfgIsSaved() {
  Config chk;
  EEPROM.get(0, chk);
  return (chk.magic == CFG_MAGIC) && (memcmp(&chk, &cfg, sizeof(Config)) == 0);
}

static void sendConfig() {
  txReset();
  txStr("K");
  txKeyI("mode", linMode);
  txKeyF("fsp",  forceSetpoint, 2);
  txKeyF("rpm",  rpmSetpoint, 1);
  txKeyI("pump", pumpPWM);
  txKeyI("lin",  manualPWM);
  txKeyF("kp",   cfg.kp, 4);
  txKeyF("ki",   cfg.ki, 4);
  txKeyF("kd",   cfg.kd, 4);
  txKeyF("omax", cfg.pid_out_max, 1);
  txKeyF("pmin", cfg.pid_min_pwm, 1);
  txKeyF("db",   cfg.pid_db, 2);
  txKeyI("fsign", cfg.f_sign);
  txKeyF("rpwm", cfg.reset_pwm, 1);
  txKeyF("dtgt", cfg.reset_target, 3);
  txKeyI("dcmp", cfg.reset_cmp);
  txKeyI("rtmo", (long)cfg.reset_timeout_ms);
  txKeyF("izero", cfg.i_zero_v, 5);
  txKeyF("ivpa", cfg.i_v_per_a, 6);
  txKeyF("vgain", cfg.v_gain, 5);
  txKeyF("voff", cfg.v_off, 5);
  txKeyF("dgain", cfg.d_gain, 5);
  txKeyF("doff", cfg.d_off, 5);
  txKeyF("fcal", cfg.f_cal, 4);
  txKeyI("foff", cfg.f_offset);
  txKeyF("falpha", cfg.f_alpha, 3);
  txKeyI("poles", cfg.poles);
  txKeyI("vpoll", cfg.vesc_poll);
  txKeyI("wd",   (long)cfg.wd_ms);
  txKeyI("dirty", cfgIsSaved() ? 0 : 1);   // 1 = calibration not yet in EEPROM
  txSend();
}


// =====================================================================
//  Command parsing
// =====================================================================

static bool applyParam(const char *k, const char *v) {
  float fv = atof(v);
  long  iv = atol(v);

  if      (!strcasecmp(k, "mode"))  { setMode((int)iv); }
  else if (!strcasecmp(k, "fsp"))   { forceSetpoint = fv; }
  else if (!strcasecmp(k, "rpm"))   { rpmSetpoint = fv; }
  else if (!strcasecmp(k, "pump"))  { setPumpPWM((int)iv); }
  else if (!strcasecmp(k, "lin"))   { manualPWM = constrain((int)iv, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
                                      if (linMode == MODE_MANUAL) setLinearPWM(manualPWM); }
  else if (!strcasecmp(k, "kp"))    { cfg.kp = fv; }
  else if (!strcasecmp(k, "ki"))    { cfg.ki = fv; }
  else if (!strcasecmp(k, "kd"))    { cfg.kd = fv; }
  else if (!strcasecmp(k, "omax"))  { cfg.pid_out_max = constrain(fv, 0.0f, (float)MOTOR_PWM_MAX); }
  else if (!strcasecmp(k, "pmin"))  { cfg.pid_min_pwm = constrain(fv, 0.0f, (float)MOTOR_PWM_MAX); }
  else if (!strcasecmp(k, "db"))    { cfg.pid_db = fabsf(fv); }
  else if (!strcasecmp(k, "fsign")) { cfg.f_sign = (iv < 0) ? -1 : 1; }
  else if (!strcasecmp(k, "rpwm"))  { cfg.reset_pwm = constrain(fv, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX); }
  else if (!strcasecmp(k, "dtgt"))  { cfg.reset_target = fv; }
  else if (!strcasecmp(k, "dcmp"))  { cfg.reset_cmp = (iv != 0) ? 1 : 0; }
  else if (!strcasecmp(k, "rtmo"))  { cfg.reset_timeout_ms = (uint32_t)iv; }
  else if (!strcasecmp(k, "izero")) { cfg.i_zero_v = fv; }
  else if (!strcasecmp(k, "ivpa"))  { cfg.i_v_per_a = (fv == 0.0f) ? 0.0125f : fv; }
  else if (!strcasecmp(k, "vgain")) { cfg.v_gain = fv; }
  else if (!strcasecmp(k, "voff"))  { cfg.v_off = fv; }
  else if (!strcasecmp(k, "dgain")) { cfg.d_gain = fv; }
  else if (!strcasecmp(k, "doff"))  { cfg.d_off = fv; }
  else if (!strcasecmp(k, "fcal"))  { cfg.f_cal = (fv == 0.0f) ? 1.0f : fv; forceFiltPrimed = false; }
  else if (!strcasecmp(k, "foff"))  { cfg.f_offset = atol(v); forceFiltPrimed = false; }
  else if (!strcasecmp(k, "falpha")){ cfg.f_alpha = constrain(fv, 0.01f, 1.0f); }
  else if (!strcasecmp(k, "poles")) { cfg.poles = (iv < 1) ? 1 : (int16_t)iv; }
  else if (!strcasecmp(k, "vpoll")) { cfg.vesc_poll = (iv != 0) ? 1 : 0; }
  else if (!strcasecmp(k, "wd"))    { cfg.wd_ms = (uint32_t)iv; }
  else return false;

  return true;
}

// Learn the current sensor's true 0 A output, with the junction open.
//
// This deliberately does NOT take its own burst of readings.  A calibration is
// only valid if it is measured exactly the way the measurement it corrects is
// measured -- same channel order, same point in the cycle, same everything --
// otherwise whatever biases the run loop is absent from the calibration and
// survives as a fixed offset.  So the zero simply averages the very rawCurrent
// values the acquisition cycle produces, over n cycles.
static void startZeroJob(uint16_t n) {
  if (n < 1) n = 1;
  if (n > 500) n = 500;
  izRemaining = n;
  izCount = 0;
  izAccum = 0;
  flags |= FLG_CALIBRATING;
  txReset();
  txStr("A zero start");
  txKeyI("n", n);
  txKeyF("ms", n * (float)CYCLE_MS, 0);
  txSend();
}

// Called once per acquisition cycle, immediately after readAnalogSensors().
static void serviceZeroJob() {
  if (!izRemaining) return;
  izAccum += (int64_t)rawCurrent;
  izCount++;
  if (--izRemaining == 0) {
    cfg.i_zero_v = ((float)((double)izAccum / (double)izCount)) * (ADC_VREF / ADC_COUNTS);
    if (!jobRemaining) flags &= ~FLG_CALIBRATING;
    txReset();
    txStr("A zero");
    txKeyI("n", izCount);
    txKeyF("izero", cfg.i_zero_v, 5);
    txSend();
    sendConfig();
  }
}

// Diagnostic: quantify how far the first conversion after a mux change sits
// from a settled one, on each channel.  A large delta on the current channel
// is what makes a naive burst-average zero disagree with the run loop.
static void adcDiag() {
  const uint8_t pins[3]  = {PIN_ADC_CURRENT_SENSE, PIN_ADC_VOLTAGE_SENSE, PIN_ADC_POSITION_SENSE};
  const char *first[3]   = {"i1", "v1", "d1"};
  const char *settled[3] = {"i2", "v2", "d2"};
  const char *delta[3]   = {"idiff", "vdiff", "ddiff"};

  txReset();
  txStr("A adcdiag");
  for (uint8_t j = 0; j < 3; j++) {
    (void)analogRead(pins[(j + 2) % 3]);      // park the mux on a different channel
    long f = analogRead(pins[j]);             // first conversion after the switch
    long s = analogRead(pins[j]);             // same channel, now settled
    txKeyI(first[j], f);
    txKeyI(settled[j], s);
    txKeyI(delta[j], f - s);
  }
  txSend();
}

static void processLine(char *line) {
  char *p = line;
  while (*p == ' ' || *p == '\t') p++;
  if (*p == '$') p++;

  // optional trailing checksum
  char *star = strchr(p, '*');
  if (star) {
    *star = 0;
    uint8_t want = (uint8_t)strtol(star + 1, NULL, 16);
    uint8_t have = 0;
    for (char *q = p; *q; q++) have ^= (uint8_t)*q;
    if (have != want) { sendErr("checksum"); return; }
  }
  if (!*p) return;

  lastHostMs = millis();
  hostSeen = true;
  if (flags & FLG_WATCHDOG) flags &= ~FLG_WATCHDOG;

  char *cmd = strtok(p, " ,\t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "SET")) {
    int n = 0, bad = 0;
    char *tok;
    while ((tok = strtok(NULL, " ,\t")) != NULL) {
      char *eq = strchr(tok, '=');
      if (!eq) { bad++; continue; }
      *eq = 0;
      if (applyParam(tok, eq + 1)) n++; else bad++;
    }
    txReset();
    txStr("A SET");
    txKeyI("n", n);
    txKeyI("bad", bad);
    txSend();
    sendConfig();
  }
  else if (!strcasecmp(cmd, "GET") || !strcasecmp(cmd, "CFG")) {
    sendConfig();
  }
  else if (!strcasecmp(cmd, "TARE")) {
    // $TARE [n=<samples>]   -- averages n conversions, one per acquisition
    // cycle, without blocking.  n=10 takes about 1.1 s of wall time.
    uint16_t n = 10;
    char *tok;
    while ((tok = strtok(NULL, " ,\t")) != NULL) {
      char *eq = strchr(tok, '=');
      if (!eq) continue;
      *eq = 0;
      if (!strcasecmp(tok, "n")) n = (uint16_t)atol(eq + 1);
    }
    startJob(JOB_TARE, n, 0.0f);
  }
  else if (!strcasecmp(cmd, "CAL")) {
    // $CAL w=<grams> [n=<samples>]  -- with a known mass resting on the cell,
    // solve for counts-per-gram.  Tare first.
    float w = 0.0f;
    uint16_t n = 10;
    char *tok;
    while ((tok = strtok(NULL, " ,\t")) != NULL) {
      char *eq = strchr(tok, '=');
      if (!eq) continue;
      *eq = 0;
      if      (!strcasecmp(tok, "w")) w = atof(eq + 1);
      else if (!strcasecmp(tok, "n")) n = (uint16_t)atol(eq + 1);
    }
    if (fabsf(w) < 1e-6f) sendErr("cal needs w=<grams>");
    else                  startJob(JOB_CAL, n, w);
  }
  else if (!strcasecmp(cmd, "ZERO")) {
    // $ZERO [n=20]  -- averages n acquisition cycles with the junction open
    uint16_t n = 20;
    char *tok;
    while ((tok = strtok(NULL, " ,\t")) != NULL) {
      char *eq = strchr(tok, '=');
      if (!eq) continue;
      *eq = 0;
      if (!strcasecmp(tok, "n")) n = (uint16_t)atol(eq + 1);
    }
    startZeroJob(n);
  }
  else if (!strcasecmp(cmd, "ADCDIAG")) {
    adcDiag();
  }
  else if (!strcasecmp(cmd, "STOP")) {
    emergencyStop();
    sendAck("stop");
    sendConfig();
  }
  else if (!strcasecmp(cmd, "SAVE")) {
    // Teensy's EEPROM.write skips bytes that already hold the right value, so
    // re-saving unchanged calibration costs no write endurance.
    EEPROM.put(0, cfg);
    if (cfgIsSaved()) {
      txReset();
      txStr("A saved");
      txKeyI("bytes", (long)sizeof(Config));
      txKeyF("fcal", cfg.f_cal, 4);
      txKeyI("foff", cfg.f_offset);
      txKeyF("izero", cfg.i_zero_v, 5);
      txSend();
    } else {
      sendErr("EEPROM verify failed - calibration NOT stored");
    }
    sendConfig();
  }
  else if (!strcasecmp(cmd, "LOAD")) {
    Config tmp;
    EEPROM.get(0, tmp);
    if (tmp.magic == CFG_MAGIC) {
      cfg = tmp;
      forceFiltPrimed = false;
      sendAck("loaded");
    } else {
      sendErr("no saved config");
    }
    sendConfig();
  }
  else if (!strcasecmp(cmd, "DEFAULTS")) {
    loadDefaults();
    forceFiltPrimed = false;
    sendAck("defaults");
    sendConfig();
  }
  else if (!strcasecmp(cmd, "PING")) {
    sendAck("pong");
  }
  else if (!strcasecmp(cmd, "ID")) {
    sendInfo("sodium-reactor teensy32 fw=1.1 proto=1");
  }
  else {
    sendErr("unknown command");
  }
}

static char rxbuf[192];
static uint16_t rxlen = 0;

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxlen > 0) {
        rxbuf[rxlen] = 0;
        processLine(rxbuf);
        rxlen = 0;
      }
    } else if (rxlen < sizeof(rxbuf) - 1) {
      rxbuf[rxlen++] = c;
    } else {
      rxlen = 0;              // overlong line, discard
      sendErr("line too long");
    }
  }
}


// =====================================================================
//  Acquisition and control
// =====================================================================

static inline float adcVolts(uint16_t raw) { return raw * (ADC_VREF / ADC_COUNTS); }

// Read a channel with a throwaway conversion first.  Switching the ADC mux
// leaves the sample-and-hold carrying charge from the previously selected
// channel, so the first conversion after a switch is pulled toward the old
// channel's voltage.  Junction voltage is the worst affected -- it is sampled
// right after the current channel, which sits near 2.5 V.
static uint16_t adcReadSettled(uint8_t pin) {
  (void)analogRead(pin);
  return (uint16_t)analogRead(pin);
}

static void readAnalogSensors() {
  rawCurrent  = adcReadSettled(PIN_ADC_CURRENT_SENSE);
  rawVoltage  = adcReadSettled(PIN_ADC_VOLTAGE_SENSE);
  rawDistance = adcReadSettled(PIN_ADC_POSITION_SENSE);

  junctionCurrent = (adcVolts(rawCurrent) - cfg.i_zero_v) / cfg.i_v_per_a;
  junctionVoltage = adcVolts(rawVoltage) * cfg.v_gain + cfg.v_off;
  junctionPower   = junctionCurrent * junctionVoltage;

  distanceVolts = adcVolts(rawDistance);
  distanceMM    = distanceVolts * cfg.d_gain + cfg.d_off;
}

static void finishJob() {
  long avg = (long)(jobAccum / (int64_t)jobCount);
  uint8_t kind = jobKind;

  jobKind = JOB_NONE;
  jobRemaining = 0;
  flags &= ~FLG_CALIBRATING;

  if (kind == JOB_TARE) {
    cfg.f_offset = avg;
    txReset();
    txStr("A tare");
    txKeyI("n", jobCount);
    txKeyI("foff", cfg.f_offset);
    txSend();
  } else {
    float counts = (float)(avg - cfg.f_offset);
    if (fabsf(jobGrams) < 1e-6f || fabsf(counts) < 1.0f) {
      sendErr("cal failed - no load change measured");
    } else {
      cfg.f_cal = counts / jobGrams;
      txReset();
      txStr("A cal");
      txKeyI("n", jobCount);
      txKeyF("fcal", cfg.f_cal, 4);
      txSend();
    }
  }
  forceFiltPrimed = false;      // zero moved, restart the display filter
  sendConfig();
}

// Start a tare (kind = JOB_TARE) or scale calibration (JOB_CAL) averaging job.
static void startJob(uint8_t kind, uint16_t n, float grams) {
  if (linMode == MODE_FORCE) {
    sendErr("refused - leave force control before taring/calibrating");
    return;
  }
  if (n < 1) n = 1;
  if (n > 500) n = 500;
  jobKind = kind;
  jobRemaining = n;
  jobCount = 0;
  jobAccum = 0;
  jobGrams = grams;
  flags |= FLG_CALIBRATING;
  txReset();
  txStr(kind == JOB_TARE ? "A tare start" : "A cal start");
  txKeyI("n", n);
  txKeyF("ms", n * (float)CYCLE_MS, 0);
  if (kind == JOB_CAL) txKeyF("w", grams, 2);
  txSend();
}

static void readForce() {
  if (hxUpdate()) {
    if (hxRaw >= HX_SAT_COUNT || hxRaw <= -HX_SAT_COUNT) flags |= FLG_FORCE_SAT;
    else                                                 flags &= ~FLG_FORCE_SAT;

    if (jobKind != JOB_NONE) {
      jobAccum += (int64_t)hxRaw;
      jobCount++;
      if (--jobRemaining == 0) finishJob();
    }
  }

  // No timeout policy is needed to read the sensor, but we do want to notice
  // if it stops converting altogether (unpowered, unplugged, DOUT stuck high).
  if (!hxHave || (millis() - hxLastMs) > HX_STALE_MS) {
    flags |= FLG_FORCE_BAD;
    forceValid = false;
    return;
  }
  flags &= ~FLG_FORCE_BAD;
  forceValid = true;

  float w = (float)(hxRaw - cfg.f_offset) / cfg.f_cal;
  if (!forceFiltPrimed) { forceG = w; forceFiltPrimed = true; }
  else                  { forceG = forceG + cfg.f_alpha * (w - forceG); }
}

static void readVesc() {
  if (!cfg.vesc_poll) {
    vescOK = false;
    flags &= ~FLG_VESC_TIMEOUT;
    vescERPM = vescRPM = 0.0f;
    vescMotorCurrent = vescInputCurrent = vescInputVoltage = 0.0f;
    return;
  }
  if (VESC.getVescValues()) {
    vescOK = true;
    flags &= ~FLG_VESC_TIMEOUT;
    vescERPM         = VESC.data.rpm;
    vescRPM          = vescERPM / (float)cfg.poles;
    vescMotorCurrent = VESC.data.avgMotorCurrent;
    vescInputCurrent = VESC.data.avgInputCurrent;
    vescInputVoltage = VESC.data.inpVoltage;
  } else {
    vescOK = false;
    flags |= FLG_VESC_TIMEOUT;
  }
}

static void commandVesc() {
  if (!cfg.vesc_poll) return;
  if (fabsf(rpmSetpoint) < 1.0f) {
    VESC.setCurrent(0.0f);              // release -- let the disc coast
  } else {
    VESC.setRPM(rpmSetpoint * (float)cfg.poles);
  }
}

static void runControl(float dt) {
  switch (linMode) {

    case MODE_FORCE: {
      if (!forceValid) { setLinearPWM(0); return; }

      pidError = forceSetpoint - forceG;
      if (fabsf(pidError) < cfg.pid_db) pidError = 0.0f;

      // derivative on measurement (no setpoint-change kick)
      float deriv = 0.0f;
      if (pidPrimed && dt > 0.0f) deriv = -(forceG - pidPrevMeas) / dt;
      pidPrevMeas = forceG;
      pidPrimed = true;

      pidIntegral += pidError * dt;
      // conditional anti-windup: keep the I contribution inside the output clamp
      if (cfg.ki != 0.0f) {
        float iLim = cfg.pid_out_max / fabsf(cfg.ki);
        pidIntegral = constrain(pidIntegral, -iLim, iLim);
      } else {
        pidIntegral = 0.0f;
      }

      float out = cfg.kp * pidError + cfg.ki * pidIntegral + cfg.kd * deriv;
      out *= (float)cfg.f_sign;

      if (fabsf(out) >= cfg.pid_out_max) flags |= FLG_PID_SAT;
      else                               flags &= ~FLG_PID_SAT;
      out = constrain(out, -cfg.pid_out_max, cfg.pid_out_max);

      // stiction floor
      if (cfg.pid_min_pwm > 0.0f && fabsf(out) > 0.5f && fabsf(out) < cfg.pid_min_pwm)
        out = (out > 0.0f) ? cfg.pid_min_pwm : -cfg.pid_min_pwm;

      pidOutput = out;
      setLinearPWM((int)lroundf(out));
      break;
    }

    case MODE_RESET: {
      bool done = cfg.reset_cmp ? (distanceMM <= cfg.reset_target)
                                : (distanceMM >= cfg.reset_target);
      if (done) {
        setLinearPWM(0);
        setMode(MODE_IDLE);              // reached target -- hold position
        sendAck("reset complete");
        sendConfig();
      } else if (cfg.reset_timeout_ms > 0 &&
                 (millis() - resetStartMs) > cfg.reset_timeout_ms) {
        setLinearPWM(0);
        setMode(MODE_IDLE);
        flags |= FLG_RESET_TIMEOUT;
        sendErr("reset timeout");
        sendConfig();
      } else {
        // reset_pwm is signed: its sign is the retract direction for this
        // machine, so it is applied as-is rather than forced negative.
        setLinearPWM((int)lroundf(cfg.reset_pwm));
      }
      break;
    }

    case MODE_MANUAL:
      setLinearPWM(manualPWM);
      break;

    case MODE_IDLE:
    default:
      setLinearPWM(0);
      pidOutput = 0.0f;
      break;
  }
}

static void checkWatchdog() {
  if (!hostSeen || cfg.wd_ms == 0) return;
  if ((millis() - lastHostMs) > cfg.wd_ms) {
    if (!(flags & FLG_WATCHDOG)) {
      flags |= FLG_WATCHDOG;
      setMode(MODE_IDLE);
      setLinearPWM(0);
      setPumpPWM(0);
      rpmSetpoint = 0.0f;
      sendErr("host watchdog - actuators stopped");
    }
  }
}

static void sendTelemetry() {
  txReset();
  txStr("D");
  txKeyI("t",    (long)millis());
  txKeyI("n",    (long)seq++);
  txKeyF("ji",   junctionCurrent, 3);
  txKeyF("jv",   junctionVoltage, 4);
  txKeyF("jw",   junctionPower, 3);
  txKeyF("f",    forceG, 2);
  txKeyF("fsp",  forceSetpoint, 2);
  txKeyF("perr", pidError, 2);
  txKeyI("fr",   hxRaw);            // raw HX711 counts, for calibration
  txKeyI("busy", (jobRemaining > izRemaining) ? jobRemaining : izRemaining);
  txKeyF("d",    distanceVolts, 4);
  txKeyF("dmm",  distanceMM, 3);
  txKeyF("rpm",  vescRPM, 1);
  txKeyF("erpm", vescERPM, 1);
  txKeyF("mi",   vescMotorCurrent, 2);
  txKeyF("ii",   vescInputCurrent, 2);
  txKeyF("iv",   vescInputVoltage, 2);
  txKeyF("rsp",  rpmSetpoint, 1);
  txKeyI("lp",   linearPWM);
  txKeyI("pmp",  pumpPWM);
  txKeyI("md",   linMode);
  txKeyI("vok",  vescOK ? 1 : 0);
  txKeyI("flg",  flags);
  txKeyI("jir",  rawCurrent);
  txKeyI("jvr",  rawVoltage);
  txKeyI("dr",   rawDistance);
  txSend();
}


// =====================================================================
void setup() {
  Serial.begin(9600);  // Speed over USB is always max hardware capability

  loadDefaults();
  bool savedCal = false;
  {
    Config tmp;
    EEPROM.get(0, tmp);
    if (tmp.magic == CFG_MAGIC) { cfg = tmp; savedCal = true; }  // use saved calibration if present
  }

  Serial1.begin(115200);  // VESC is 115200 baud.  Hardware serial port.
  while (!Serial1) {;}
  VESC.setSerialPort(&Serial1); //Define which ports to use as UART

  hxBegin();  //  The HX711 has data available every 100ms.  We poll DOUT once per
              //  110ms cycle and clock the waiting sample out; nothing blocks.


  analogReadResolution(ADC_BITS);
  analogReadAveraging(32);  // Conversion time for 16 bit, 32 averages is about 100 microseconds


  pinMode(PIN_LINEAR_FWD, OUTPUT);
  pinMode(PIN_LINEAR_REV, OUTPUT);
  pinMode(PIN_LINEAR_EN, OUTPUT);

  pinMode(PIN_WATER_PUMP_SUPPLY, OUTPUT);
  pinMode(PIN_WATER_PUMP_GND, OUTPUT);
  pinMode(PIN_WATER_PUMP_EN, OUTPUT);

  analogWriteFrequency(PIN_LINEAR_FWD, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_LINEAR_REV, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_WATER_PUMP_SUPPLY, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_WATER_PUMP_GND, MOTOR_PWM_FREQUENCY);

  analogWriteResolution(MOTOR_PWM_BITS);
  analogWrite(PIN_LINEAR_FWD, 0);
  analogWrite(PIN_LINEAR_REV, 0);
  analogWrite(PIN_WATER_PUMP_SUPPLY, 0);
  analogWrite(PIN_WATER_PUMP_GND, 0);

  digitalWrite(PIN_LINEAR_EN, 1);
  digitalWrite(PIN_WATER_PUMP_EN, 1);

  lastCycleMs = millis();
  lastHostMs  = millis();

  sendInfo("sodium-reactor teensy32 fw=1.1 proto=1");

  // With no stored calibration, zero the load cell over the first ~1.1 s.
  // A saved tare offset is trusted and left alone.
  if (!savedCal) startJob(JOB_TARE, 10, 0.0f);

  sendConfig();
}


void loop() {
  //Check USB serial for commands (water pump PWM, VESC RPM, modes, setpoints...)
  //Actuator outputs are only changed when a new value arrives over USB.
  handleSerial();

  uint32_t now = millis();
  if ((int32_t)(now - lastCycleMs) < CYCLE_MS) return;

  float dt = (now - lastCycleMs) / 1000.0f;
  lastCycleMs += CYCLE_MS;
  if ((int32_t)(now - lastCycleMs) > (int32_t)(4 * CYCLE_MS)) lastCycleMs = now;  // resync after a stall

  //Send updated RPM value to VESC (repeated every cycle: the VESC times out
  //its own command after ~1s and would otherwise stop on its own)
  commandVesc();

  //Collect values from VESC:  RPM, avg motor current, avg input current, inpVoltage
  readVesc();

  //Analog read of junction voltage and current, and of the optical distance sensor
  readAnalogSensors();
  serviceZeroJob();   //feeds a running $ZERO average with this cycle's reading

  //Read HX711 force sensor
  readForce();

  //Closed-loop force regulation / reset-retract / manual drive
  runControl(dt);

  checkWatchdog();

  //Send everything over USB serial
  sendTelemetry();
}
