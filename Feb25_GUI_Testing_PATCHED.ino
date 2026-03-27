#include <JrkG2.h>

// ====== SELECT THE SERIAL PORT TO THE JRK ======
#define JRK_PORT Serial1
JrkG2Serial jrk(JRK_PORT);

// ====== USER KNOBS ======
const uint32_t JRK_BAUD = 100000;   // Must match Jrk "UART, fixed baud rate"
bool systemRunning = false;

float    INHALE_FRAC = 0.40;
float    HOLD_FRAC   = 0.10;

float    BPM = 25.0;             // breaths per minute
uint16_t CENTER = 2048;          // nominal mid-point
uint16_t AMPL   = 400;           // counts (0..)
float    MAXDUTY_FRAC = 0.35;    // 0.05..1.0

const uint32_t LOG_DT_MS = 20;   // 50 Hz logging
bool stopOnHaltingError = true;

// ====== SAFETY SETTINGS (YOU SHOULD SET THESE) ======
// Safe motion window in Jrk scaled units (0..4095). Set based on your calibration.
const uint16_t FB_SAFE_MIN = 50;     // TODO: set to your safe min
const uint16_t FB_SAFE_MAX = 4045;   // TODO: set to your safe max

// Communication watchdog: if no valid command received within this time, stop.
const uint32_t HOST_TIMEOUT_MS = 4000;

// Optional overcurrent trip in mA (set to 0 to disable)
const uint16_t CURRENT_TRIP_MA = 0;  // TODO: e.g. 3000 if you want protection

// ====== INTERNALS ======
uint32_t cycleStartMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastCmdMs = 0;

String lineBuf;

// Pending updates applied at cycle boundary
bool pendingUpdate = false;
float pendingBPM = 50.0;
uint16_t pendingAMPL = 750;
float pendingMaxDutyFrac = 0.35;

static uint16_t clampTargetRange(int32_t t)
{
  if (t < 0) return 0;
  if (t > 4095) return 4095;
  return (uint16_t)t;
}

static uint16_t clampToSafeWindow(uint16_t tgt)
{
  if (tgt < FB_SAFE_MIN) return FB_SAFE_MIN;
  if (tgt > FB_SAFE_MAX) return FB_SAFE_MAX;
  return tgt;
}

// ----- Jrk max duty helpers -----
void applyMaxDuty(uint16_t d)
{
  // Jrk duty is non-negative for limits; set both directions.
  if (d > 600) d = 600;  // 600 is common full scale for Jrk duty units

  jrk.setMaxDutyCycleForward(d);
  jrk.setMaxDutyCycleReverse(d);

  Serial.print(F("# OK MaxDutyUnits="));
  Serial.println(d);
}

void applyMaxDutyFrac(float frac)
{
  if (frac < 0.05f) frac = 0.05f;
  if (frac > 1.0f)  frac = 1.0f;
  uint16_t d = (uint16_t)(frac * 600.0f + 0.5f);
  MAXDUTY_FRAC = frac;           // keep state truthful
  applyMaxDuty(d);
}

// ----- Safety actions -----
void goSafeHoldCenter()
{
  uint16_t safeCenter = clampToSafeWindow(CENTER);
  jrk.setTarget(safeCenter);
}

void emergencyStop(const __FlashStringHelper* reason)
{
  jrk.stopMotor();
  Serial.print(F("# STOP: "));
  Serial.println(reason);
}

// ----- Host command handling -----
void noteCommandReceived()
{
  lastCmdMs = millis();
}

void requestUpdate(float bpm, uint16_t ampl, float maxDutyFrac)
{
  pendingBPM = bpm;
  pendingAMPL = ampl;
  pendingMaxDutyFrac = maxDutyFrac;
  pendingUpdate = true;
}

void parseCommand(const String &line)
{
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Single-word commands
  if (s.equalsIgnoreCase("STOP"))
  {
    systemRunning = false;
    emergencyStop(F("STOP command"));
    noteCommandReceived();
    return;
  }
  if (s.equalsIgnoreCase("START"))
  {
    systemRunning = true;
    goSafeHoldCenter();   // optional: move safely before resuming
    Serial.println(F("# STARTED"));
    noteCommandReceived();
    return;
  }
  if (s.equalsIgnoreCase("GET"))
  {
    Serial.print(F("STATE,BPM=")); Serial.print(BPM, 2);
    Serial.print(F(",AMPL=")); Serial.print(AMPL);
    Serial.print(F(",MAXDUTYFRAC=")); Serial.println(MAXDUTY_FRAC, 3);
    noteCommandReceived();
    return;
  }

  int sp = s.indexOf(' ');
  if (sp < 0) { Serial.println(F("# ERR bad format")); return; }

  String key = s.substring(0, sp);
  String valS = s.substring(sp + 1);
  key.toUpperCase();

  if (key == "BPM")
  {
    float v = valS.toFloat();
    if (v < 1) v = 1;
    if (v > 150) v = 150;

    requestUpdate(v, AMPL, MAXDUTY_FRAC);
    Serial.print(F("# PENDING BPM=")); Serial.println(v, 2);
    noteCommandReceived();
  }
  else if (key == "AMPL")
  {
    long v = valS.toInt();
    if (v < 0) v = 0;
    if (v > 1500) v = 1500;

    requestUpdate(BPM, (uint16_t)v, MAXDUTY_FRAC);
    Serial.print(F("# PENDING AMPL=")); Serial.println((uint16_t)v);
    noteCommandReceived();
  }
  else if (key == "MAXDUTY")
  {
    // Interpret values <= 1.2 as fraction, otherwise as raw duty units.
    float v = valS.toFloat();
    float frac = MAXDUTY_FRAC;

    if (v <= 1.2f)
    {
      frac = v;
      if (frac < 0.05f) frac = 0.05f;
      if (frac > 1.0f)  frac = 1.0f;
      requestUpdate(BPM, AMPL, frac);
      Serial.print(F("# PENDING MAXDUTYFRAC=")); Serial.println(frac, 3);
    }
    else
    {
      // raw units (0..600)
      long du = (long)v;
      if (du < 0) du = 0;
      if (du > 600) du = 600;
      frac = ((float)du) / 600.0f;
      requestUpdate(BPM, AMPL, frac);
      Serial.print(F("# PENDING MAXDUTYUNITS=")); Serial.println((uint16_t)du);
    }
    noteCommandReceived();
  }
  else
  {
    Serial.println(F("# ERR unknown key"));
  }
}

void handleHostCommands()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      lineBuf.trim();
      if (lineBuf.length() > 0) parseCommand(lineBuf);
      lineBuf = "";
    }
    else
    {
      if (lineBuf.length() < 80) lineBuf += c;
    }
  }
}

void safetyWatchdog()
{
  if (millis() - lastCmdMs > HOST_TIMEOUT_MS)
  {
    // If host goes silent, stop and hold center (prevents “runaway breathing”)
    emergencyStop(F("host timeout"));
    goSafeHoldCenter();
    lastCmdMs = millis(); // avoid spamming STOP continuously
  }
}

void setup()
{
  Serial.begin(115200);
  JRK_PORT.begin(JRK_BAUD);

  delay(250);

  // Initialize command timestamp
  lastCmdMs = millis();

  // Apply initial max duty limit in Jrk RAM
  applyMaxDutyFrac(MAXDUTY_FRAC);

  // Start at safe center
  goSafeHoldCenter();

  Serial.println(F("ms,phase,target,scaledFeedback,current_mA,errHalting,errOccurred"));
  //Serial.println(F("ms,phase,target,scaledFeedback,dutyTarget,Duty,fwDutymx,rvDutymx,current_mA,errHalting,errOccurred"));

  cycleStartMs = millis();
  lastLogMs = millis();
}

void loop()
{
  handleHostCommands();
  safetyWatchdog();

  const uint32_t now = millis();

  // ----- Compute breath timing -----
  const float cycleMsF = (60.0f / BPM) * 1000.0f;
  const uint32_t cycleMs = (uint32_t)(cycleMsF + 0.5f);

  const uint32_t inhaleMs = (uint32_t)(cycleMsF * INHALE_FRAC + 0.5f);
  const uint32_t holdMs   = (uint32_t)(cycleMsF * HOLD_FRAC   + 0.5f);
  const uint32_t exhaleMs = cycleMs - inhaleMs - holdMs;

  uint32_t t = now - cycleStartMs;
  if (t >= cycleMs)
  {
    // new cycle boundary
    cycleStartMs = now;
    t = 0;

    // Apply pending updates only at cycle boundary (smoother + safer)
    if (pendingUpdate)
    {
      BPM = pendingBPM;
      AMPL = pendingAMPL;
      applyMaxDutyFrac(pendingMaxDutyFrac);
      pendingUpdate = false;

      Serial.print(F("# APPLIED BPM=")); Serial.print(BPM, 2);
      Serial.print(F(" AMPL=")); Serial.print(AMPL);
      Serial.print(F(" MAXDUTYFRAC=")); Serial.println(MAXDUTY_FRAC, 3);
    }
  }

  if (!systemRunning)
  {
    goSafeHoldCenter();
    return;  // skip waveform generation
  }
  // ----- Generate trapezoid waveform -----
  uint16_t target = CENTER;

  enum Phase { INHALE=0, HOLD=1, EXHALE=2 };
  Phase phase = INHALE;

  if (t < inhaleMs)
  {
    phase = INHALE;
    float u = (inhaleMs > 0) ? (float)t / (float)inhaleMs : 1.0f;
    target = clampTargetRange((int32_t)CENTER + (int32_t)(u * AMPL));
  }
  else if (t < inhaleMs + holdMs)
  {
    phase = HOLD;
    target = clampTargetRange((int32_t)CENTER + AMPL);
  }
  else
  {
    phase = EXHALE;
    uint32_t te = t - inhaleMs - holdMs;
    float u = (exhaleMs > 0) ? (float)te / (float)exhaleMs : 1.0f;
    target = clampTargetRange((int32_t)CENTER + (int32_t)((1.0f - u) * AMPL));
  }

  // Safety clamp to known safe mechanical window
  target = clampToSafeWindow(target);

  // Send target to Jrk
  jrk.setTarget(target);

  // ----- Read + log at fixed rate -----
  if (now - lastLogMs >= LOG_DT_MS)
  {
    lastLogMs = now;

    uint16_t scaledFb = jrk.getScaledFeedback();
    //int16_t  dutyTgt  = jrk.getDutyCycleTarget();
    uint16_t current  = jrk.getCurrent();
    uint16_t errH     = jrk.getErrorFlagsHalting();
    uint16_t errO     = jrk.getErrorFlagsOccurred();
    //int16_t Duty      = jrk.getDutyCycle();
    //uint16_t fwDutymx = jrk.getMaxDutyCycleForward();
    //uint16_t rvDutymx = jrk.getMaxDutyCycleReverse();

    Serial.print(now);
    Serial.print(',');
    Serial.print((int)phase);
    Serial.print(',');
    Serial.print(target);
    Serial.print(',');
    Serial.print(scaledFb);
    Serial.print(',');
    //Serial.print(dutyTgt);
    //Serial.print(',');
    //Serial.print(Duty);
    //Serial.print(',');
    //Serial.print(fwDutymx);
    //Serial.print(',');
    //Serial.print(rvDutymx);
    //Serial.print(',');
    Serial.print(current);
    Serial.print(',');
    Serial.print(errH, HEX);
    Serial.print(',');
    Serial.println(errO, HEX);

    // Optional overcurrent protection
    if (CURRENT_TRIP_MA > 0 && current >= CURRENT_TRIP_MA)
    {
      emergencyStop(F("overcurrent"));
      while (1) { delay(1000); }
    }

    // Jrk halting errors
    if (stopOnHaltingError && errH != 0)
    {
      emergencyStop(F("halting error"));
      while (1) { delay(1000); }
    }
  }
}
