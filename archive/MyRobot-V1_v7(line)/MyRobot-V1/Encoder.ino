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
 * Wheel SPEED is measured from the timestamps of the last few edges (see edgeSpeedMms()), not by counting
 * ticks per 50 ms, which is far too coarse at low speed. updateEncoders() feeds it to the speed controller
 * in SpeedControl.ino.
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

// Speed measurement
const unsigned long ENC_INTERVAL_MS = 50;   // how often speeds/odometry/speed-control update
const uint8_t       SPD_EDGES       = 4;    // speed = distance over the last 4 edges / the time they took (fewer if slow)
const unsigned long SPD_SPAN_MAX_US = 300000; // an edge span longer than this means the wheel is stopped/restarting -> ignore

// Straight-driving tuning (speeds in mm/s)
const float STRAIGHT_KP       = 1.5;        // mm/s of wheel-speed difference per mm of A-vs-B distance mismatch
const float STRAIGHT_MAX_TRIM = 60.0;       // max difference between the two wheel speed targets, mm/s. The lagging wheel is
                                            // sped up first; the leading wheel is only slowed if there is no headroom, never below V_MIN_MMS
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
const uint8_t    TICK_RING = 8;                       // edge timestamps kept per wheel (must be a power of 2)
volatile unsigned long tickT[2][TICK_RING];           // micros() of the most recent edges. [0] = A, [1] = B

long   posA = 0, posB = 0;          // signed ticks (direction-aware), since last resetOdometry()
int8_t dirA = 1, dirB = 1;          // last known direction of each wheel (+1 fwd, -1 back)
float  vRight = 0, vLeft = 0;       // measured wheel speeds, mm/s (signed). This is what the speed controller uses.

float odoX = 0, odoY = 0;           // mm
float odoTheta = 0;                 // radians, + = turning left (counter-clockwise)
float odoDist  = 0;                 // mm, signed distance travelled by the robot's centre

unsigned long encLastRun = 0;
long prevRawA = 0, prevRawB = 0;

// ---------------------------------------------------------------------
// Counting (interrupt)
// ---------------------------------------------------------------------

/** One ISR per encoder; each counts both edges of its pin and timestamps them (for speed measurement). */
void ENC_ISR isrEncA() { tickT[0][rawTicksA & (TICK_RING - 1)] = micros(); rawTicksA++; }
void ENC_ISR isrEncB() { tickT[1][rawTicksB & (TICK_RING - 1)] = micros(); rawTicksB++; }

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

/**
 * Speed of one wheel (unsigned, mm/s) from the timestamps of its last few encoder edges.
 * Counting ticks per 50 ms is far too coarse at low speed (with 3 mm ticks you'd see only 1-2 ticks at
 * 100 mm/s); timing the edges gives a usable reading even at the robot's slowest speed.
 * @param w 0 = motor A (right), 1 = motor B (left)
 */
float edgeSpeedMms(int w) {
    unsigned long t[SPD_EDGES + 1];
    long n;
    noInterrupts();                                   // copy the timestamps atomically
    n = (w == 0) ? rawTicksA : rawTicksB;
    for (int k = 0; k <= SPD_EDGES; k++) t[k] = tickT[w][(n - 1 - k) & (TICK_RING - 1)];   // t[0] = newest edge
    interrupts();
    unsigned long nowUs = micros();                   // read AFTER the copy so it can't be older than t[0]

    long maxK = min((long)SPD_EDGES, n - 1);
    float v = 0;
    for (long k = maxK; k >= 1; k--) {                // widest window first; fall back to fewer edges if it spans too long
        unsigned long span = t[0] - t[k];
        if (span > 0 && span <= SPD_SPAN_MAX_US) {
            v = k * MM_PER_TICK * 1000000.0f / span;
            break;
        }
    }
    if (v <= 0) return 0;

    unsigned long since = nowUs - t[0];
    if (since > SPD_SPAN_MAX_US) return 0;            // no edge for a long time: stopped
    if (since > 0) v = fminf(v, MM_PER_TICK * 1000000.0f / since);   // next edge is overdue -> can't be faster than this
    return fminf(v, 2.0f * V_MAX_MMS);                // clamp noise
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

    dirA = commandedDir(in1, in2, dirA);
    dirB = commandedDir(in3, in4, dirB);
    long sdA = dirA * dA;                    // signed ticks this interval
    long sdB = dirB * dB;
    posA += sdA;
    posB += sdB;

    // Wheel speeds from the edge timestamps, signed by the commanded direction, lightly filtered
    vRight = 0.5f * vRight + 0.5f * (dirA * edgeSpeedMms(0));    // motor A = RIGHT wheel
    vLeft  = 0.5f * vLeft  + 0.5f * (dirB * edgeSpeedMms(1));    // motor B = LEFT wheel

    // Differential-drive odometry
    float dR = sdA * MM_PER_TICK;
    float dL = sdB * MM_PER_TICK;
    float dC = 0.5 * (dR + dL);
    float dTheta = (dR - dL) / TRACK_MM;
    odoX     += dC * cos(odoTheta + 0.5 * dTheta);
    odoY     += dC * sin(odoTheta + 0.5 * dTheta);
    odoTheta += dTheta;
    odoDist  += dC;

    runSpeedControl(dt);                     // fresh speeds in -> new PWM out (SpeedControl.ino)
}

/** Zero the signed tick counts and the pose. */
void resetOdometry() {
    posA = posB = 0;
    odoX = odoY = odoTheta = odoDist = 0;
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
 * @brief Drive straight at a given speed by cancelling the distance mismatch between the two wheels.
 *        Each wheel's speed is held by the speed controller; this adds a small speed trim on top.
 * @param speed Wanted speed, mm/s, V_MIN_MMS..V_MAX_MMS (below V_MIN_MMS the robot is stopped)
 * @param dir   +1 forward, -1 backward
 * Call as often as you like; it only sets targets.
 */
void driveStraight(float speed, int dir) {
    if (speed < V_MIN_MMS) { stopRobot(); return; }   // can't hold a speed that low; also avoids driving one wheel alone

    // Ticks each wheel has travelled in the direction of travel since the drive began
    long a = dir * (posA - strBaseA);
    long b = dir * (posB - strBaseB);
    long err = constrain(a - b, -STRAIGHT_MAX_ERR, STRAIGHT_MAX_ERR);   // + = motor A is ahead

    // Total speed difference wanted, split so the commanded speed is never undercut if avoidable:
    // speed up the lagging wheel first, slow the leading wheel only for what's left (never below V_MIN_MMS).
    float corr  = fminf(STRAIGHT_KP * fabsf((float)err) * MM_PER_TICK, STRAIGHT_MAX_TRIM);
    float boost = fminf(corr, fmaxf(V_MAX_MMS - speed, 0.0f));
    float cut   = fminf(corr - boost, fmaxf(speed - V_MIN_MMS, 0.0f));
    float lag   = speed + boost;
    float lead  = speed - cut;
    float tA    = (err > 0) ? lead : lag;             // A ahead -> A is the leader ...
    float tB    = (err > 0) ? lag  : lead;            // ... and B gets the boost

    setWheelSpeeds(dir * tB, dir * tA);               // (left = B, right = A)
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
