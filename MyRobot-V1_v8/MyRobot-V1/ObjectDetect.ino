/**
 * @file ObjectDetect.ino
 * @brief Sweep the front ultrasonic sensor (mounted on a servo), pick the best object, turn to it and drive up to it.
 *
 * Sequence (all non-blocking, call objectDetect() every pass of loop()):
 *
 *   1. SWEEP     Robot stands still. The servo steps across +/- OD_SWEEP_HALF_DEG in OD_STEP_DEG steps; at every step
 *                the sensor takes 3 pings and keeps the median. Result: one distance per bearing.
 *   2. CHOOSE    Neighbouring readings at similar distances are grouped into one object. Each object gets a bearing,
 *                a distance and an estimated WIDTH (from the angle it covers and how far away it is).
 *                Objects that are too narrow (noise) or too wide (a wall) are ignored. The rest are scored:
 *
 *                    score = widthMm - OD_DIST_WEIGHT * distanceMm
 *
 *                so BIGGER objects win, and between similar-sized objects the CLOSER one wins.
 *   3. TURN      Servo goes back to centre, the robot spins in place (encoder-measured) to face the object.
 *   4. APPROACH  Drive straight (encoder-balanced) and slow down near the object; stop OD_STOP_CM away.
 *                If the object isn't where the sweep said it was, the robot stops and sweeps again.
 *
 * If a sweep finds nothing, the robot turns OD_RESCAN_TURN_DEG and sweeps again (up to OD_MAX_RESCANS times).
 *
 * Bearings: 0 = straight ahead, + = LEFT, - = RIGHT (degrees).
 *
 * File order matters: Arduino joins tabs alphabetically and a global must be declared before it is used, so this tab
 * (which needs Motor.ino's globals) sorts AFTER Motor.ino, and RunMaze.ino (which needs this tab) sorts after this one.
 */

#include <Servo.h>

Servo scanServo;

// ---------------------------------------------------------------------
// Tuning (all starting guesses: TUNE on the real robot)
// ---------------------------------------------------------------------
// --- Servo ---
const int OD_SERVO_CENTER   = 90;     // servo angle that points the sensor straight ahead
const int OD_ANGLE_SIGN     = +1;     // +1: servo angles ABOVE centre look LEFT. Set to -1 if the sweep appears mirrored.
const int OD_SWEEP_HALF_DEG = 60;     // sweep from -60 to +60 degrees (servo limits are 0..180)
const int OD_STEP_DEG       = 4;      // degrees between readings (smaller = better size estimate, slower sweep)
const int OD_N              = (2 * OD_SWEEP_HALF_DEG) / OD_STEP_DEG + 1;   // number of readings per sweep (31)
const unsigned long OD_FIRST_SETTLE_MS = 450;   // wait for the servo to swing from centre to the first angle
const unsigned long OD_SETTLE_MS       = 70;    // wait after each step before pinging
const unsigned long OD_PING_GAP_MS     = 30;    // gap between the 3 pings at one angle (lets echoes die out)

// --- Which readings count as an object ---
const float OD_MIN_RANGE_CM     = 4.0;    // closer than this = ignore (sensor blind zone)
const float OD_MAX_RANGE_CM     = 90.0;   // farther than this = not an object (arena walls, empty space)
const float OD_CLUSTER_JUMP_CM  = 8.0;    // neighbouring readings further apart than this are different objects
const int   OD_MIN_SAMPLES      = 2;      // an object must show up in at least this many neighbouring readings
const float OD_MIN_WIDTH_MM     = 15.0;   // narrower than this = noise
const float OD_MAX_WIDTH_MM     = 250.0;  // wider than this = a wall, not an object                                    TUNE
// The ultrasonic beam is a cone (~15 deg), so every object looks a little wider than it is. That is fine: it is the
// same for all objects, and only the ORDER of the sizes matters.

// --- Choosing between objects ---
const float OD_DIST_WEIGHT = 0.25;        // mm of "width" an object loses per mm of distance. 0 = pick purely by size.
                                          // 0.25: an object 40 mm wider beats one that is 160 mm closer.              TUNE

// --- Turning / approaching ---
const float OD_MIN_TURN_DEG     = 3.0;    // already facing it closely enough: don't turn
const float OD_TURN_COAST_DEG   = 3.0;    // stop spinning this many degrees early (the robot coasts a little)
const unsigned long OD_TURN_TIMEOUT_MS = 3500;
const float OD_STOP_CM          = 8.0;    // final distance from the object                                             TUNE
const float OD_SLOW_CM          = 25.0;   // start slowing down this far from the object (V_MIN_MMS at OD_STOP_CM)
const float OD_LOST_MARGIN_CM   = 25.0;   // front reading this much farther than expected = the object isn't ahead
const int   OD_LOST_COUNT       = 3;      // ... this many readings in a row = give up this approach and sweep again
const float OD_OVERSHOOT_CM     = 20.0;   // drove this much farther than the object was seen = it isn't there
const int   OD_MAX_RETRIES      = 2;      // re-sweeps after a failed approach
const float OD_RESCAN_TURN_DEG  = 90.0;   // nothing found: turn this much (left) and sweep again
const int   OD_MAX_RESCANS      = 3;      // 3 x 90 deg + the +/-60 deg sweep covers the full circle
const unsigned long OD_APPROACH_INTERVAL_MS = 60;

// ---------------------------------------------------------------------
// Public results of objectDetect()
// ---------------------------------------------------------------------
const int OD_RUNNING = 0, OD_REACHED = 1, OD_FAILED = -1;

// ---------------------------------------------------------------------
// State (plain ints, not an enum: see note in Motor.ino)
// ---------------------------------------------------------------------
const int OD_SWEEP = 0, OD_TURN = 1, OD_APPROACH = 2, OD_RESCAN_TURN = 3, OD_DONE = 4, OD_FAIL = 5;
int   odState        = OD_DONE;
float odScan[OD_N];                 // distance (cm) at each bearing; 999 = no echo
int   odIdx          = 0;           // which bearing is being measured
int   odPings        = 0;           // pings taken at this bearing
float odPingBuf[3];
unsigned long odWaitUntil = 0;
unsigned long odStateStart = 0;
unsigned long odLastRun   = 0;
float odTargetBearing = 0;          // deg, + = left
float odTargetCm      = 0;          // distance measured during the sweep
float odFront         = -1;
int   odLost          = 0;
int   odRescans       = 0;
int   odRetries       = 0;

// ---------------------------------------------------------------------
// Servo helpers
// ---------------------------------------------------------------------
/** Call once from setup(). */
void setupScanServo() {
    scanServo.attach(servoPin);
    scanServoTo(0);
}

/** Point the sensor at a bearing in degrees (0 = ahead, + = left). */
void scanServoTo(int bearingDeg) {
    scanServo.write(constrain(OD_SERVO_CENTER + OD_ANGLE_SIGN * bearingDeg, 0, 180));
}

/** Point the sensor straight ahead (wall following and follow-me expect this). */
void scanServoCenter() {
    scanServoTo(0);
}

// ---------------------------------------------------------------------
// Sweep analysis
// ---------------------------------------------------------------------
float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }     // now a <= b
    if (b > c) b = fmaxf(a, c);                   // c is smaller than b: median is the larger of a and c
    return b;
}

bool odValid(float d) {
    return d >= OD_MIN_RANGE_CM && d <= OD_MAX_RANGE_CM;
}

/**
 * Group the sweep into objects and choose the best one.
 * @return true if a target was chosen (odTargetBearing / odTargetCm are set)
 */
bool odChooseTarget() {
    bool  found     = false;
    float bestScore = -1e9f;
    int i = 0;

    Serial.print("scan(cm)=");
    for (int k = 0; k < OD_N; k++) { Serial.print((int)odScan[k]); Serial.print(k < OD_N - 1 ? ',' : '\n'); }

    while (i < OD_N) {
        if (!odValid(odScan[i])) { i++; continue; }

        // Grow a cluster while the next reading is valid and at about the same distance
        int   start = i, j = i;
        float sum = odScan[i];
        while (j + 1 < OD_N && odValid(odScan[j + 1]) && fabsf(odScan[j + 1] - odScan[j]) <= OD_CLUSTER_JUMP_CM) {
            j++;
            sum += odScan[j];
        }
        int   count   = j - start + 1;
        float distCm  = sum / count;
        float spanDeg = count * OD_STEP_DEG;                                  // angle this object covers
        float widthMm = 2.0f * distCm * 10.0f * tanf(0.5f * spanDeg * DEG_TO_RAD);
        float bearing = -OD_SWEEP_HALF_DEG + 0.5f * (start + j) * OD_STEP_DEG;   // centre of the cluster

        bool ok = (count >= OD_MIN_SAMPLES) && widthMm >= OD_MIN_WIDTH_MM && widthMm <= OD_MAX_WIDTH_MM;
        float score = widthMm - OD_DIST_WEIGHT * distCm * 10.0f;

        Serial.print("object bearing="); Serial.print(bearing);
        Serial.print(" dist=");          Serial.print(distCm);
        Serial.print(" width=");         Serial.print(widthMm);
        Serial.print(" n=");             Serial.print(count);
        Serial.print(ok ? " score=" : " (ignored) score=");
        Serial.println(score);

        if (ok && score > bestScore) {
            bestScore = score;
            odTargetBearing = bearing;
            odTargetCm = distCm;
            found = true;
        }
        i = j + 1;
    }

    if (found) {
        Serial.print("target bearing="); Serial.print(odTargetBearing);
        Serial.print(" dist=");          Serial.println(odTargetCm);
    } else {
        Serial.println("no object found");
    }
    return found;
}

// ---------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------
void odStartSweep() {
    stopRobot();
    odIdx = 0;
    odPings = 0;
    scanServoTo(-OD_SWEEP_HALF_DEG);
    odWaitUntil = millis() + OD_FIRST_SETTLE_MS;
    odState = OD_SWEEP;
    odStateStart = millis();
    Serial.println("object detect: sweeping");
}

void odBeginApproach() {
    stopRobot();
    markMotion();
    resetStraight();
    odLost = 0;
    odFront = -1;
    odLastRun = 0;
    odState = OD_APPROACH;
    odStateStart = millis();
    Serial.println("object detect: approaching");
}

void odBeginTurn() {
    scanServoCenter();
    markMotion();
    odState = OD_TURN;
    odStateStart = millis();
    if (fabsf(odTargetBearing) < OD_MIN_TURN_DEG) odBeginApproach();
}

/** Call when starting object detection (command change). */
void resetObjectDetect() {
    odRescans = 0;
    odRetries = 0;
    odStartSweep();
}

/** The approach failed or nothing was found: sweep again (or give up). */
void odAfterMiss(bool nothingFound) {
    if (nothingFound) {
        if (odRescans >= OD_MAX_RESCANS) { odState = OD_FAIL; stopRobot(); Serial.println("object detect: FAILED, nothing found"); return; }
        odRescans++;
        scanServoCenter();
        markMotion();
        odState = OD_RESCAN_TURN;
        odStateStart = millis();
        Serial.println("object detect: nothing here, turning to look elsewhere");
    } else {
        if (odRetries >= OD_MAX_RETRIES) { odState = OD_FAIL; stopRobot(); Serial.println("object detect: FAILED, lost the object"); return; }
        odRetries++;
        odStartSweep();
    }
}

// ---------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------
/**
 * @brief Run object detection. Call every pass of loop() while in this mode.
 * @param speed approach speed, mm/s (the robot slows down near the object)
 * @return OD_RUNNING, OD_REACHED (stopped next to the object) or OD_FAILED
 */
int objectDetect(float speed) {
    unsigned long now = millis();

    if (odState == OD_SWEEP) {
        if (now < odWaitUntil) return OD_RUNNING;

        float d = readDistanceCm(trigF, echoF);
        odPingBuf[odPings++] = (d < 0) ? 999.0f : d;          // no echo = far away
        if (odPings < 3) {
            odWaitUntil = now + OD_PING_GAP_MS;
            return OD_RUNNING;
        }

        odScan[odIdx] = median3(odPingBuf[0], odPingBuf[1], odPingBuf[2]);
        odPings = 0;
        odIdx++;
        if (odIdx < OD_N) {
            scanServoTo(-OD_SWEEP_HALF_DEG + odIdx * OD_STEP_DEG);
            odWaitUntil = now + OD_SETTLE_MS;
            return OD_RUNNING;
        }

        // Sweep finished
        scanServoCenter();
        if (odChooseTarget()) odBeginTurn();
        else                  odAfterMiss(true);
    }

    else if (odState == OD_TURN || odState == OD_RESCAN_TURN) {
        // Spin in place by the wanted angle (measured by the encoders), then approach / sweep again
        bool  rescan = (odState == OD_RESCAN_TURN);
        float wanted = rescan ? OD_RESCAN_TURN_DEG : fabsf(odTargetBearing);
        bool  left   = rescan ? true : (odTargetBearing > 0);
        if (turnedDeg() >= wanted - OD_TURN_COAST_DEG || now - odStateStart > OD_TURN_TIMEOUT_MS) {
            stopRobot();
            if (rescan) odStartSweep();
            else        odBeginApproach();
        } else if (left) {
            spinLeft(CORNER_SPIN_MMS);
        } else {
            spinRight(CORNER_SPIN_MMS);
        }
    }

    else if (odState == OD_APPROACH) {
        if (now - odLastRun < OD_APPROACH_INTERVAL_MS) return OD_RUNNING;
        odLastRun = now;

        float front    = readDistanceCm(trigF, echoF);
        float travelCm = travelledMm() / 10.0f;
        float remainCm = odTargetCm - travelCm;               // where the sweep says the object still is

        // Arrived
        if (front > 0 && front <= OD_STOP_CM) {
            stopRobot();
            odState = OD_DONE;
            Serial.print("object detect: reached, front="); Serial.println(front);
            return OD_REACHED;
        }

        // Is the object still ahead of us? (reading much farther than expected, or no echo, several times in a row)
        bool off = (front < 0) || (front > remainCm + OD_LOST_MARGIN_CM);
        if (off) odLost++; else odLost = 0;
        if (odLost >= OD_LOST_COUNT || travelCm > odTargetCm + OD_OVERSHOOT_CM) {
            stopRobot();
            Serial.println("object detect: object not where expected");
            odAfterMiss(false);
            return (odState == OD_FAIL) ? OD_FAILED : OD_RUNNING;
        }

        // Slow down smoothly: full speed at OD_SLOW_CM, V_MIN_MMS at OD_STOP_CM
        float top = fmaxf(speed, V_MIN_MMS);
        float s = top;
        if (front > 0 && front < OD_SLOW_CM) {
            float frac = constrain((front - OD_STOP_CM) / (OD_SLOW_CM - OD_STOP_CM), 0.0f, 1.0f);
            s = V_MIN_MMS + (top - V_MIN_MMS) * frac;
        }
        driveStraight(s, +1);

        Serial.print("approach front="); Serial.print(front);
        Serial.print(" expected=");      Serial.println(remainCm);
    }

    if (odState == OD_DONE) return OD_REACHED;
    if (odState == OD_FAIL) return OD_FAILED;
    return OD_RUNNING;
}
