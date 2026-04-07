#include <JrkG2.h>

// ====== SELECT THE SERIAL PORT TO THE JRK ======
#define JRK_PORT Serial1
JrkG2Serial jrk(JRK_PORT);

// ====== Globals ========
bool DEBUG_STEP_MODE = false;
bool debugPauseOnStateEntry = false;

// ====== USER KNOBS ======
const uint32_t JRK_BAUD = 100000;   // Must match Jrk "UART, fixed baud rate"
bool systemRunning = false;

float    INHALE_FRAC  = 0.2500f;      
float    HOLD_FRAC    = 0.2500f;
float    HOLD_EX_FRAC = 0.5000f;         

float    BPM = 60.0;             // breaths per minute
uint16_t CENTER = 2048;          // nominal mid-point
uint16_t AMPL   = 500;           // counts (0..)
float    MAXDUTY_FRAC = 0.5;    // 0.05..1.0

const uint32_t LOG_DT_MS = 20;   // 50 Hz logging
bool stopOnHaltingError = true;

// ====== SAFETY SETTINGS (YOU SHOULD SET THESE) ======
// Safe motion window in Jrk scaled units (0..4095). Set based on your calibration.
const uint16_t FB_SAFE_MIN = 50;     // TODO: set to your safe min
const uint16_t FB_SAFE_MAX = 4045;   // TODO: set to your safe max

// Communication watchdog: if no valid command received within this time, stop.
const uint32_t HOST_TIMEOUT_MS = 0;
const bool USE_HOST_WATCHDOG = false; 

// Optional overcurrent trip in mA (set to 0 to disable)
const uint16_t CURRENT_TRIP_MA = 0;  // TODO: e.g. 3000 if you want protection

// ====== INTERNALS ======
uint32_t lastLogMs = 0;
uint32_t lastCmdMs = 0;

String lineBuf;

// State timeout and dwell
uint32_t stateEntryMs = 0;

const uint16_t ENDPOINT_TOL = 20;   // tune this

bool pendingUpdate = false;
float pendingBPM = 50.0;
uint16_t pendingAMPL = 750;
float pendingMaxDutyFrac = 0.35;

// Debug helper


// States
typedef enum {
    IDLE,
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE_RAMP,
    EXHALE_HOLD,
    FAULT
} State;

State currentState = IDLE;

// Logging States
const char* stateToString(State s) {
    switch (s) {
        case IDLE: return "IDLE";
        case INHALE_RAMP: return "INHALE_RAMP";
        case INHALE_HOLD: return "INHALE_HOLD";
        case EXHALE_RAMP: return "EXHALE_RAMP";
        case EXHALE_HOLD: return "EXHALE_HOLD";
        case FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

// Ramping Target Helper
uint16_t lerpTarget(uint16_t startVal, uint16_t endVal, uint32_t elapsedMs, uint32_t durMs)
{
  if (durMs == 0) return endVal;

  float u = (float)elapsedMs / (float)durMs;
  u = constrain(u, 0.0f, 1.0f);

  float y = (1.0f - u) * (float)startVal + u * (float)endVal;
  return (uint16_t)y;
}

// Safety for Target
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

// ------- Debug Step Helper ------------
void waitForSerialStep(const char* label)
{
  if (!DEBUG_STEP_MODE) return;

  Serial.print(F("# STEP: "));
  Serial.println(label);
  Serial.println(F("# Send 'next' <Control-space> to continue"));

  while (true)
  {
    handleHostCommands();   // keep serial parser alive

    if (Serial.available())
    {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.equalsIgnoreCase("n") || s.equalsIgnoreCase("next") || s.equalsIgnoreCase("c"))
      {
        break;
      }
    }
  }
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

// ----- State helpers / internals -
void enterState(State newState)
{
  currentState = newState;
  stateEntryMs = millis();

  // Apply pending updates at the start of a new inhale cycle
  if (newState == INHALE_RAMP && pendingUpdate)
  {
    BPM = pendingBPM;
    AMPL = pendingAMPL;
    applyMaxDutyFrac(pendingMaxDutyFrac);
    pendingUpdate = false;

    Serial.print(F("# APPLIED BPM=")); Serial.print(BPM, 2);
    Serial.print(F(" AMPL=")); Serial.print(AMPL);
    Serial.print(F(" MAXDUTYFRAC=")); Serial.println(MAXDUTY_FRAC, 3);
  }

  if (debugPauseOnStateEntry)
  {
    waitForSerialStep(stateToString(newState));
    stateEntryMs = millis();  // restart timing after the pause
  }
}

static bool endpointReached(uint16_t fb, uint16_t endpoint, uint16_t tol)
{
  return abs((int32_t)fb - (int32_t)endpoint) <= (int32_t)tol;
}
// ----- State function -----------
uint16_t updateStateMachine(uint16_t scaledFb,
                            uint32_t inhaleMs,
                            uint32_t holdMs,
                            uint32_t exRampMs,
                            uint32_t exHoldMs,
                            uint8_t &phase)
{
  const uint16_t inhaleEndpoint =
      clampToSafeWindow(clampTargetRange((int32_t)CENTER + (int32_t)AMPL));

  const uint16_t exhaleEndpoint =
      clampToSafeWindow(CENTER);

  uint32_t elapsed = millis() - stateEntryMs;

  // If not running, just hold center / exhale endpoint
  if (!systemRunning)
  {
    if (currentState != IDLE) enterState(IDLE);
    phase = 1;   // treat as hold for logging
    return exhaleEndpoint;
  }

  // Auto-start state machine from IDLE
  if (currentState == IDLE)
  {
    enterState(INHALE_RAMP);
  }

  uint16_t target = exhaleEndpoint;

  switch (currentState)
  {
    case INHALE_RAMP:
      phase = 0;                 // inhale

      if (elapsed >= inhaleMs)
      {
        enterState(INHALE_HOLD);
        target = inhaleEndpoint; // safe target for this loop
      }
      else if (endpointReached(scaledFb, inhaleEndpoint, ENDPOINT_TOL))
      {
        // Arrived early: hold at endpoint until inhaleMs expires
        target = inhaleEndpoint;
      }
      else
      {
        // Normal ramp toward endpoint 
        target = lerpTarget(exhaleEndpoint, inhaleEndpoint, elapsed, inhaleMs);
      }
      break;

    case INHALE_HOLD:
      phase = 1;                 // hold
      target = inhaleEndpoint;

      if (elapsed >= holdMs)
      {
        enterState(EXHALE_RAMP);
      }
      break;

    case EXHALE_RAMP:
      phase = 2;                 // exhale

      if (elapsed >= exRampMs)
      {
        enterState(EXHALE_HOLD);
        target = exhaleEndpoint;  // safe target for this loop
      }
      else if (endpointReached(scaledFb, exhaleEndpoint, ENDPOINT_TOL))
      {
        // Arrived early: hold at endpoint until exRampMs expires
        target = exhaleEndpoint;

      }
      else
      {
        // Normal ramp back toward exhale endpoint
        target = lerpTarget(inhaleEndpoint, exhaleEndpoint, elapsed, exRampMs);
      }
      break;  

    case EXHALE_HOLD:
      phase = 1;                 // hold
      target = exhaleEndpoint;

      if (elapsed >= exHoldMs)
      {
        enterState(INHALE_RAMP);
      }
      break;

    case FAULT:
      phase = 1;
      target = exhaleEndpoint;
      break;

    default:
      phase = 1;
      target = exhaleEndpoint;
      enterState(IDLE);
      break;
  }

  return clampToSafeWindow(target);
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
    enterState(IDLE);
    emergencyStop(F("STOP command"));
    noteCommandReceived();
    return;
  }
  if (s.equalsIgnoreCase("START"))
  {
    systemRunning = true;
    enterState(INHALE_RAMP);
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
  if (!USE_HOST_WATCHDOG) return; 

  if (millis() - lastCmdMs > HOST_TIMEOUT_MS)
  {
    systemRunning = false; 
    enterState(IDLE);
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

  Serial.println(F("ms,state,target,scaledFeedback,error,current_mA,dutyTarget,dutyActual,errHalting,errOccurred"));

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

  const uint32_t exHoldMs = (uint32_t)(exhaleMs * HOLD_EX_FRAC + 0.5f);
  const uint32_t exRampMs = exhaleMs - exHoldMs;

 

  // ----- Run target waveform ---------
  uint16_t scaledFb = jrk.getScaledFeedback();

  uint8_t phase = 1;
  uint16_t target = updateStateMachine(scaledFb, inhaleMs, holdMs, exRampMs, exHoldMs, phase);

  jrk.setTarget(target);

  // ----- Read + log at fixed rate -----
  if (now - lastLogMs >= LOG_DT_MS)
  {
    lastLogMs = now;

    int16_t  dutyTgt = jrk.getDutyCycleTarget();
    int16_t  dutyAct = jrk.getDutyCycle();
    int16_t  error   = (int16_t)target - (int16_t)scaledFb;
    uint16_t current = jrk.getCurrent();
    uint16_t errH    = jrk.getErrorFlagsHalting();
    uint16_t errO    = jrk.getErrorFlagsOccurred();

    Serial.print(now);
    Serial.print(',');
    Serial.print(stateToString(currentState));
    Serial.print(',');
    Serial.print(target);
    Serial.print(',');
    Serial.print(scaledFb);
    Serial.print(',');
    Serial.print(error);
    Serial.print(',');
    Serial.print(current);
    Serial.print(',');
    Serial.print(dutyTgt);
    Serial.print(',');
    Serial.print(dutyAct);
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
