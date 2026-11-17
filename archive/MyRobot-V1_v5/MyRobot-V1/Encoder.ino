/**
 * @file Encoder.ino
 * @brief Wheel encoder counting, speed, odometry, and encoder-based straight driving.
 *
 * One single-channel encoder per motor:
 *   Motor A (RIGHT wheel) encoder -> pin 3
 *   Motor B (LEFT wheel)  encoder -> pin 2
 *
 * Portable version: each encoder pin gets its own attachInterrupt() on CHANGE, so it
 * works on non-AVR boards (Uno R4, ESP32, RP2040, SAMD, ...). The ISRs only count edges;
 * everything else happens in updateEncoders(), which loop() must call every pass.
 *
 * Single-channel = no direction signal. Direction is taken from the H-bridge pins (what we
 * COMMANDED), so right after a direction flip, while a wheel is still coasting the old way,
 * a few ticks get the wrong sign. Fine for this robot; a quadrature encoder would fix it.
 */

// ESP32 requires ISRs to live in IRAM; other boards don't care.
#if defined(ESP32)
  #define ENC_ISR IRAM_ATTR
#else
  #define ENC_ISR
#endif

// ---------------------------------------------------------------------
// Calibrate these (see notes in the chat)
// ---------------------------------------------------------------------
const float MM_PER_TICK = 3.0;     // wheel travel per counted edge   (TODO: calibrate)
const float TRACK_MM    = 140.0;   // distance between wheel contact points (TODO: measure)

// Straight-driving tuning
const unsigned long ENC_INTERVAL_MS = 50;   // how often speeds/odometry/straight-correction update
const float STRAIGHT_KP       = 3.0;        // PWM counts of correction per tick of A-vs-B mismatch
const int   STRAIGHT_MAX_TRIM = 40;         // max total PWM difference between the wheels. The lagging wheel is boosted first;
                                            // the leading wheel is only cut if there is no headroom, and never below MIN_PWM
const long  STRAIGHT_MAX_ERR  = 15;         // ticks; clamps the mismatch so a stalled wheel can't wind it up

// ---------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------
// Uno R4 WiFi: only pins 2 and 3 are documented external-interrupt pins, so the encoders live there.
// (Motor driver IN1/IN2 moved to pins 13/12 in MyRobot-V1.ino to free them up.)
const uint8_t ENC_A_PIN = 3;    // motor A encoder
const uint8_t ENC_B_PIN = 2;    // motor B encoder

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------
volatile long    rawTicksA = 0;     // ISR-owned: unsigned edge counts since boot
volatile long    rawTicksB = 0;

long   posA = 0, posB = 0;          // signed ticks (direction-aware), since last resetOdometry()
int8_t dirA = 1, dirB = 1;          // last known direction of each wheel (+1 fwd, -1 back)
float  vRight = 0, vLeft = 0;       // filtered wheel speeds, mm/s (signed)

float odoX = 0, odoY = 0;           // mm
float odoTheta = 0;                 // radians, + = turning left (counter-clockwise)
float odoDist  = 0;                 // mm, signed distance travelled by the robot's centre

unsigned long encLastRun = 0;
long prevRawA = 0, prevRawB = 0;
unsigned long lastMoveA = 0, lastMoveB = 0;   // last time each wheel produced a tick (for stall detection)

// ---------------------------------------------------------------------
// Counting (interrupt)
// ---------------------------------------------------------------------

/** One ISR per encoder; each counts both edges of its pin. */
void ENC_ISR isrEncA() { rawTicksA++; }
void ENC_ISR isrEncB() { rawTicksB++; }

/** Call once from setup(). */
void setupEncoders() {
    pinMode(ENC_A_PIN, INPUT_PULLUP);   // harmless for push-pull sensors, required for open-collector ones
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isrEncA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), isrEncB, CHANGE);
    encLastRun = millis();
}

// ---------------------------------------------------------------------
// Direction, speed, odometry (main loop)
// ---------------------------------------------------------------------

/** Direction we commanded, read back from the H-bridge pins (matches drive()/reverse() in Motor.ino). */
int8_t commandedDir(int pinX, int pinY, int8_t last) {
    bool x = digitalRead(pinX);
    bool y = digitalRead(pinY);
    if (!x && y) return +1;    // drive():   IN1 LOW,  IN2 HIGH
    if (x && !y) return -1;    // reverse(): IN1 HIGH, IN2 LOW
    return last;               // stopped / coasting: assume it keeps rolling the way it was going
}

/** Call every pass of loop(). Does its work every ENC_INTERVAL_MS. */
void updateEncoders() {
    unsigned long now = millis();
    if (now - encLastRun < ENC_INTERVAL_MS) return;
    float dt = (now - encLastRun) / 1000.0;
    encLastRun = now;

    long rawA, rawB;
    noInterrupts();                          // multi-byte reads aren't atomic on small MCUs
    rawA = rawTicksA;
    rawB = rawTicksB;
    interrupts();
    long dA = rawA - prevRawA;  prevRawA = rawA;
    long dB = rawB - prevRawB;  prevRawB = rawB;
    if (dA > 0) lastMoveA = now;
    if (dB > 0) lastMoveB = now;

    dirA = commandedDir(in1, in2, dirA);
    dirB = commandedDir(in3, in4, dirB);
    long sdA = dirA * dA;                    // signed ticks this interval
    long sdB = dirB * dB;
    posA += sdA;
    posB += sdB;

    // Wheel speeds (low-pass filtered; one tick is quantisation noise at low speed)
    float instR = sdA * MM_PER_TICK / dt;    // motor A = RIGHT wheel
    float instL = sdB * MM_PER_TICK / dt;    // motor B = LEFT wheel
    vRight = 0.6 * vRight + 0.4 * instR;
    vLeft  = 0.6 * vLeft  + 0.4 * instL;

    // Differential-drive odometry
    float dR = sdA * MM_PER_TICK;
    float dL = sdB * MM_PER_TICK;
    float dC = 0.5 * (dR + dL);
    float dTheta = (dR - dL) / TRACK_MM;
    odoX     += dC * cos(odoTheta + 0.5 * dTheta);
    odoY     += dC * sin(odoTheta + 0.5 * dTheta);
    odoTheta += dTheta;
    odoDist  += dC;
}

/** Zero the signed tick counts and the pose. */
void resetOdometry() {
    posA = posB = 0;
    odoX = odoY = odoTheta = odoDist = 0;
}

// ---------------------------------------------------------------------
// Stall detection: a wheel that is being driven but produces no ticks
// ---------------------------------------------------------------------

/** Restart the stall timers. Call when a new motion begins. */
void resetStall() {
    lastMoveA = lastMoveB = millis();
}

/** True if either wheel has produced no encoder ticks for `ms`. Only meaningful while BOTH wheels are being driven. */
bool wheelsStalled(unsigned long ms) {
    unsigned long now = millis();
    return (now - lastMoveA > ms) || (now - lastMoveB > ms);
}

// ---------------------------------------------------------------------
// Distance / angle since a marker (replaces "drive for N ms" / "spin for N ms")
// ---------------------------------------------------------------------
float markDist = 0, markTheta = 0;

/** Remember where we are; travelledMm()/turnedDeg() measure from here. */
void markMotion() {
    markDist  = odoDist;
    markTheta = odoTheta;
}

/** Distance driven since markMotion(), mm (magnitude). */
float travelledMm() { return fabs(odoDist - markDist); }

/** Angle turned since markMotion(), degrees (magnitude). */
float turnedDeg()   { return fabs(odoTheta - markTheta) * 57.2958; }

// ---------------------------------------------------------------------
// Straight driving: keep both wheels' tick counts equal
// ---------------------------------------------------------------------
long strBaseA = 0, strBaseB = 0;

/** Call when a straight drive starts (command changes to W or S). */
void resetStraight() {
    strBaseA = posA;
    strBaseB = posB;
}

/**
 * @brief Drive straight by cancelling the tick mismatch between the two wheels.
 * @param speed Base PWM, MIN_PWM-255 (below MIN_PWM the robot is stopped)
 * @param dir   +1 forward, -1 backward
 * Call every pass of loop(); it rate-limits itself.
 */
void driveStraight(int speed, int dir) {
    static unsigned long lastRun = 0;
    unsigned long now = millis();
    if (now - lastRun < ENC_INTERVAL_MS) return;
    lastRun = now;

    if (speed < MIN_PWM) { stopRobot(); return; }   // can't move the robot; also avoids driving one wheel alone

    // Ticks each wheel has travelled in the direction of travel since the drive began
    long a = dir * (posA - strBaseA);
    long b = dir * (posB - strBaseB);
    long err = constrain(a - b, -STRAIGHT_MAX_ERR, STRAIGHT_MAX_ERR);   // + = motor A is ahead

    // Total PWM difference wanted, split so the commanded speed is never undercut if avoidable:
    // boost the lagging wheel first, cut the leading wheel only for what's left (never below MIN_PWM).
    int corr  = min((int)(STRAIGHT_KP * labs(err)), STRAIGHT_MAX_TRIM);
    int boost = min(corr, 255 - speed);
    int cut   = min(corr - boost, speed - MIN_PWM);
    int lag   = speed + boost;
    int lead  = speed - cut;
    int pwmA  = (err > 0) ? lead : lag;           // A ahead -> A is the leader ...
    int pwmB  = (err > 0) ? lag  : lead;          // ... and B gets the boost

    driveSigned(in1, in2, enA, dir * pwmA);
    driveSigned(in3, in4, enB, dir * pwmB);
}

// ---------------------------------------------------------------------
// Telemetry (call at <= 5 Hz; Serial is only 9600 baud)
// ---------------------------------------------------------------------
void printEncoders() {
    Serial.print("encA=");  Serial.print(posA);
    Serial.print(" encB="); Serial.print(posB);
    Serial.print(" vR=");   Serial.print(vRight);
    Serial.print(" vL=");   Serial.print(vLeft);
    Serial.print(" x=");    Serial.print(odoX);
    Serial.print(" y=");    Serial.print(odoY);
    Serial.print(" th=");   Serial.println(odoTheta * 57.2958);
}

/** Debug: raw interrupt counts and the live pin levels. Use this if the counts stay at 0. */
void printRawEncoders() {
    Serial.print("rawA=");    Serial.print(rawTicksA);
    Serial.print(" rawB=");   Serial.print(rawTicksB);
    Serial.print(" pinA=");  Serial.print(digitalRead(ENC_A_PIN));
    Serial.print(" pinB=");  Serial.println(digitalRead(ENC_B_PIN));
}
