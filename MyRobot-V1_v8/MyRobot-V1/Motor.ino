/**
 * @file Motor.ino
 * @brief Simple H-bridge motor control helpers + autonomous modes.
 *
 * Provides a minimal interface for driving DC motors using an L298N
 * (Arduino-style platforms).
 *
 * Direction is controlled via two input pins, and speed is controlled
 * by PWM on the enable pin (analogWrite, 0-255).
 *
 * Low-level functions take pins as parameters (one motor at a time).
 * High-level functions (driveForward, turnLeft, ...) use the global pin
 * variables defined in MyRobot-V1.ino and control both motors together.
 *
 * Motor A = RIGHT wheel, Motor B = LEFT wheel.
 *
 * ALL speeds in this file are in mm/s (signed, + = forward). They are TARGETS for the closed-loop wheel speed
 * controller in SpeedControl.ino, which uses the encoders to find the PWM. PWM only appears in the low-level
 * drive()/reverse()/stop()/driveSigned() helpers below, which that controller calls.
 *
 * MIN_PWM (MyRobot-V1.ino) is the lowest PWM that actually moves the robot; weaker only makes the motors hum.
 *
 * Outside corner: drive past it, spin right in place, drive straight to the new wall.
 * Corner / wall logic uses the wheel encoders (see Encoder.ino) for distance and angle
 * instead of timers: travelledMm(), turnedDeg(), markMotion(). Timeouts are kept only as
 * a safety net in case an encoder stops working.
 *
 * @author Paul Bucci
 * @date 2026
 */


// ---------------------------------------------------------------------
// Low-level: one motor at a time
// ---------------------------------------------------------------------

/**
 * @brief Drives a DC motor in the "forward" direction at a given speed.
 *
 * @param in1   GPIO pin connected to motor driver input 1 (direction control)
 * @param in2   GPIO pin connected to motor driver input 2 (direction control)
 * @param enA   PWM-capable pin connected to motor driver enable pin
 * @param speed Speed from 0 (off) to 255 (full)
 */
void drive(int in1, int in2, int enA, int speed) {
    digitalWrite(in1, LOW);                      // Direction control: IN1
    digitalWrite(in2, HIGH);                     // Direction control: IN2
    analogWrite(enA, constrain(speed, 0, 255));  // Speed via PWM
}

/**
 * @brief Drives a DC motor in the "backward" direction at a given speed.
 *        (Same as drive(), but with the direction pins flipped.)
 */
void reverse(int in1, int in2, int enA, int speed) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, constrain(speed, 0, 255));
}

/**
 * @brief Stops a DC motor.
 */
void stop(int in1, int in2, int enA) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);                         // Disable motor driver
}

/**
 * @brief Drives a motor with a signed speed.
 *        Positive = forward, negative = backward.
 *        |speed| below MIN_PWM can't move the robot (the motor only hums), so it stops instead.
 *
 * @param speed Signed speed from -255 to 255
 */
void driveSigned(int in1, int in2, int enA, int speed) {
    if (speed >= MIN_PWM) {
        drive(in1, in2, enA, speed);
    } else if (speed <= -MIN_PWM) {
        reverse(in1, in2, enA, -speed);
    } else {
        stop(in1, in2, enA);
    }
}


// ---------------------------------------------------------------------
// High-level: both wheels. Speeds are mm/s TARGETS for the closed-loop speed
// controller (SpeedControl.ino), not PWM. A wheel asked for less than V_MIN_MMS
// (but not 0) is stopped, because the robot can't hold a speed that low.
// ---------------------------------------------------------------------

/** @brief Stop both motors and switch the speed controller off. */
void stopRobot() {
    stopWheelControl();
    stop(in1, in2, enA);
    stop(in3, in4, enB);
}

/** @brief Drive both wheels forward at `speed` mm/s. */
void driveForward(float speed) {
    setWheelSpeeds(speed, speed);
}

/** @brief Drive both wheels backward at `speed` mm/s. */
void driveBackward(float speed) {
    setWheelSpeeds(-speed, -speed);
}

/**
 * @brief Turn right while driving forward: right wheel slower than left.
 * @param speed Speed of the outside (left) wheel, mm/s
 * @param turn  How much to slow the inside (right) wheel, mm/s. Bigger = sharper.
 *              If speed - turn is below V_MIN_MMS the inside wheel stops (robot pivots on it).
 */
void turnRight(float speed, float turn) {
    setWheelSpeeds(speed, speed - turn);     // (left, right)
}

/** @brief Turn left while driving forward: left wheel slower than right. */
void turnLeft(float speed, float turn) {
    setWheelSpeeds(speed - turn, speed);
}

/** @brief Turn right while backing up. */
void turnRightBackward(float speed, float turn) {
    setWheelSpeeds(-speed, -(speed - turn));
}

/** @brief Turn left while backing up. */
void turnLeftBackward(float speed, float turn) {
    setWheelSpeeds(-(speed - turn), -speed);
}

/** @brief Spin in place to the right: right wheel back, left wheel forward, each at `speed` mm/s. */
void spinRight(float speed) {
    setWheelSpeeds(speed, -speed);
}

/** @brief Spin in place to the left: right wheel forward, left wheel back, each at `speed` mm/s. */
void spinLeft(float speed) {
    setWheelSpeeds(-speed, speed);
}

// ---------------------------------------------------------------------
// Sensing + autonomous modes
// ---------------------------------------------------------------------
unsigned long FOLLOW_INTERVAL_MS = 60;

// Follow-me
float FOLLOW_DIST_CM  = 15.0;
float DEADBAND_CM     = 3.0;
float MAX_RANGE_CM    = 100.0;
float FOLLOW_KP       = 3.0;    // extra mm/s per cm of error BEYOND the deadband, added on top of V_MIN_MMS
                                // (speed runs from V_MIN_MMS at the deadband edge up to the commanded speed)

// Wall following (all speeds in mm/s; base speed >= V_MIN_MMS)
float WALL_KP        = 6.5;     // mm/s of wheel-speed difference per cm of wall-distance error
float WALL_KD        = 10.0;    // mm/s per unit wall SLOPE (cm of wall-distance change per cm travelled x10, ~tan of the
                                // angle to the wall), measured from the encoders. Speed-independent, unlike a per-second rate.
float MAX_CORR       = 90.0;    // max steering AWAY from the wall, mm/s. The slow wheel is clamped at V_MIN_MMS and the extra
                                // differential comes from speeding the fast wheel up (capped at V_MAX_MMS).
float MAX_CORR_TOWARD = 90.0;   // max steering TOWARD the wall (convex curves need more authority)
float FRONT_STOP_CM  = 15.0;    // stop distance when (almost) standing still. It grows with MEASURED speed (STOP_CM_PER_MMS).
float STOP_CM_PER_MMS = 0.03;   // extra stop distance, cm per mm/s of measured speed: reaction time + coasting
float FRONT_STOP_MAX_CM = 22.0; // cap on the speed-scaled stop distance; keep it below FRONT_CLEAR_CM
int   MAX_MISSES     = 8;
float WALL_MIN_DS_MM = 8.0;     // need at least this much travel between slope samples (a tick is only a few mm)

// Corner handling
float FRONT_CLEAR_CM     = 25.0;  // inside corner: front must read > this to stop turning (hysteresis vs FRONT_STOP_CM)
float FRONT_SLOW_CM      = 40.0;  // start slowing down when a wall is this close ahead, at standstill. Speed is ramped down to V_MIN_MMS at the stop distance.
float SLOW_CM_PER_MMS    = 0.08;  // extra slow-down start distance, cm per mm/s of measured speed
float WALL_REACQUIRE_CM  = 15.0;  // inside corner: right sensor must read < target + this to resume following
float OUTSIDE_JUMP_CM    = 25.0;  // reading jumps this far in ONE step vs. the filtered value = wall just ended (sharp outside corner)
float OUTSIDE_LOST_CM    = 60.0;  // reading this far beyond target = wall is gone (gradual loss); also lost after MAX_MISSES no-echoes
float OUTSIDE_FAR_CM     = 12.0;  // right reading this far beyond target counts as "lost" (no echo also counts as lost)
float WALL_FOUND_CM      = 10.0;  // after an outside-corner turn: wall found when right sensor reads < target + this, twice in a row

// --- Encoder-based distances / angles (these replaced the *_MS timers; all are starting guesses, TUNE) ---
float OUTSIDE_LOST_MM    = 40.0;  // right sensor lost the wall for this much TRAVEL = wall has ended      (was OUTSIDE_LOST_MS 250)
// Outside corner = (1) drive past the corner, (2) spin right IN PLACE, (3) drive straight until the new wall is seen.
// (A gentle arc can't work: the inner wheel can't go below V_MIN_MMS, so the arc would be far too wide.)
// Step 1 distance is computed from the wall standoff: the wheel axle must be one standoff past the corner before turning.
float OUTSIDE_FWD_OFFSET_MM = 0.0;    // extra forward travel: how far the right sensor sits AHEAD of the wheel axle, plus clearance. TUNE.
float OUTSIDE_FWD_MIN_MM    = 20.0;   // never drive less than this after the wall is lost
float OUTSIDE_TURN_DEG      = 85.0;   // spin right this many degrees (a bit under 90 to allow for coasting). TUNE; depends on TRACK_MM.
float OUTSIDE_FIND_MAX_MM   = 400.0;  // give up looking for the new wall after this much straight travel
float MIN_SPIN_DEG       = 30.0;  // always spin at least this many degrees (ignores a noisy early 'clear') (was MIN_SPIN_MS 250)
float EXTRA_SPIN_DEG     = 6.0;   // keep spinning this many degrees AFTER the corner looks clear           (was EXTRA_SPIN_MS 80)
float MAX_SPIN_DEG       = 150.0; // give up the spin after this many degrees
float POST_TURN_SETTLE_MM = 150.0;// after any corner, limit steering for this much travel                 (was POST_TURN_SETTLE_MS 800)

// --- Safety-net timeouts (only matter if an encoder dies) ---
unsigned long INSIDE_TIMEOUT_MS      = 3000;  // give up spinning after this long
unsigned long OUTSIDE_TIMEOUT_MS     = 4000;  // give up the outside turn / wall search after this long
unsigned long OUTSIDE_FWD_TIMEOUT_MS = 4000;  // give up the straight run after this long

// --- Speeds, mm/s ---
float CORNER_SPIN_MMS = 120.0;   // wheel speed while spinning in place at inside AND outside corners (>= V_MIN_MMS).
                                 // The speed controller adds whatever PWM it takes to hold this, even against the extra torque a spin needs.
float SETTLE_MAX_CORR = 30.0;    // max steering (either way, mm/s) during the post-corner settling distance

// Captured wall target
float TARGET_MIN_CM  = 8.0;
float TARGET_MAX_CM  = 40.0;
const int CAL_SAMPLES = 5;

// Wall-follow state (global so it can be reset)
float wallTarget  = -1;        // -1 = not captured yet
float wfFiltered  = -1;
float wfRate      = 0;         // wall slope (see WALL_KD)
float wfPrevD     = -1;        // filtered wall distance at the last slope sample (-1 = none yet)
float wfPrevDist  = 0;         // odoDist at the last slope sample
float wfLastGood  = -1;
int   wfMisses    = 0;
int   wfCalCount  = 0;
float wfCalSum    = 0;
unsigned long wfLastRun = 0;

// (plain ints, not an enum: Arduino's auto-generated prototypes can't see enum types across .ino tabs)
const int WF_FOLLOW = 0, WF_INSIDE_TURN = 1, WF_OUTSIDE_FWD = 2, WF_OUTSIDE_FIND = 3, WF_OUTSIDE_TURN = 4;
int wfState = WF_FOLLOW;
unsigned long wfStateStart = 0;
int   wfFoundCount = 0;          // consecutive 'new wall found' readings during the arc
bool  wfLost = false;            // right sensor currently doesn't see the wall
float wfLostAtMm = 0;            // odoDist when the wall was first lost
int   wfNearStreak = 0;          // consecutive good wall readings (a stray echo must not cancel 'lost')
float outsideFwdMm = 0;          // how far to drive past the corner before turning (computed when the wall is lost)
float wfSettleMm = 0;            // gentle-steering distance after a corner ends (0 = not settling)
float wfClearDeg = -1;           // degrees turned when the inside-corner turn first looked finished (-1 = not yet)

/** @brief Distance in cm from ultrasonic sensor, or -1 on no echo */
float readDistanceCm(int trig, int echo) {
    digitalWrite(trig, LOW);    delayMicroseconds(2);
    digitalWrite(trig, HIGH);   delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, 25000);  // 25 ms timeout
    if (dur == 0) return -1;
    return dur * 0.0343 / 2.0;
}

/** @brief Follow whatever is in front of the robot. Call often from loop().
 *         Uses the encoder-balanced straight driver so it doesn't veer while approaching/backing off. */
void followMe(float maxSpeed) {
    static unsigned long lastRun = 0;
    static float filtered = -1;
    static int lastDir = 0;      // 0 = stopped, +1 = forward, -1 = backward (restart the straight baseline on change)

    if (millis() - lastRun < FOLLOW_INTERVAL_MS) return;
    lastRun = millis();

    float d = readDistanceCm(trigF, echoF);
    if (d < 0 || d > MAX_RANGE_CM) {      // nothing in range: stop and wait
        filtered = -1;
        lastDir = 0;
        stopRobot();
        return;
    }

    filtered = (filtered < 0) ? d : 0.5 * filtered + 0.5 * d;

    float error = filtered - FOLLOW_DIST_CM;   // + = too far, - = too close
    Serial.print("front="); Serial.println(filtered);

    if (fabs(error) < DEADBAND_CM) {
        lastDir = 0;
        stopRobot();
        return;
    }

    // V_MIN_MMS at the edge of the deadband, rising with distance error up to the commanded speed
    float top   = fmaxf(maxSpeed, V_MIN_MMS);
    float power = V_MIN_MMS + FOLLOW_KP * (fabsf(error) - DEADBAND_CM);
    power = constrain(power, V_MIN_MMS, top);

    int dir = (error > 0) ? +1 : -1;
    if (dir != lastDir) {                 // (re)starting or reversing: forget old wheel mismatch
        resetStraight();
        lastDir = dir;
    }
    driveStraight(power, dir);
}

/** @brief Call when wall following starts; the next readings set the target. */
void resetWallFollow() {
    wallTarget = -1;
    wfFiltered = -1;
    wfRate     = 0;
    wfPrevD    = -1;
    wfLastGood = -1;
    wfMisses   = 0;
    wfCalCount = 0;
    wfCalSum   = 0;
    wfLastRun  = millis();
    wfState    = WF_FOLLOW;
    wfStateStart = millis();
    wfSettleMm = 0;
    wfLost     = false;
    wfNearStreak = 0;
    wfFoundCount = 0;
    wfClearDeg = -1;
    markMotion();
    resetStraight();                 // the target-capture run is driven straight
}

void enterWallState(int s) {
    wfState = s;
    wfStateStart = millis();
    markMotion();                    // turnedDeg() / travelledMm() now measure from here
    wfClearDeg = -1;
    wfLost = false;
    wfNearStreak = 0;
    wfFoundCount = 0;
    if (s == WF_OUTSIDE_FWD || s == WF_OUTSIDE_FIND) resetStraight();
    if (s == WF_FOLLOW) {
        wfSettleMm = POST_TURN_SETTLE_MM;
        wfLastGood = -1;     // forget the OLD wall's distance, or the first reading on the new wall looks like a huge jump
    }
    // Reset the PD memory so we don't carry a stale derivative into FOLLOW
    wfFiltered = -1;
    wfRate = 0;
    wfPrevD = -1;
    wfMisses = 0;
    Serial.print("wallState="); Serial.println(s);
}

/** @brief Follow a wall on the robot's RIGHT, holding the distance it started at.
 *         Handles gradual corners (PD), inside corners and outside corners.
 *         Corners are measured in encoder distance / angle, not time. */
void straightFollow(float speed) {
    unsigned long now = millis();
    if (now - wfLastRun < FOLLOW_INTERVAL_MS) return;
    wfLastRun = now;

    float front = readDistanceCm(trigF, echoF);
    float d     = readDistanceCm(trigR, echoR);

    // Stop / slow-down distances grow with how fast we are ACTUALLY going (encoder speed), so a faster robot reacts earlier.
    // (Spinning in place averages to ~0, so corner turns use the base distances.)
    float vNow    = fabsf(0.5f * (vRight + vLeft));                                       // mm/s
    float stopCm  = fminf(FRONT_STOP_CM + STOP_CM_PER_MMS * vNow, FRONT_STOP_MAX_CM);
    float slowCm  = FRONT_SLOW_CM + SLOW_CM_PER_MMS * vNow;
    bool frontBlocked = (front > 0 && front < stopCm);

    // ---------------- Corner states ----------------
    if (wfState == WF_INSIDE_TURN) {
        // Spin left (away from the right-hand wall) until front is clear AND
        // the right sensor sees the new wall at a sensible distance.
        float turned    = turnedDeg();
        bool frontClear = (front < 0 || front > FRONT_CLEAR_CM);
        bool wallSeen   = (d > 0 && d < wallTarget + WALL_REACQUIRE_CM);
        // Latch the angle at which the turn first looks done, then keep spinning EXTRA_SPIN_DEG more
        if (wfClearDeg < 0 && frontClear && wallSeen && turned > MIN_SPIN_DEG) {
            wfClearDeg = turned;
        }
        bool finished = (wfClearDeg >= 0 && turned - wfClearDeg >= EXTRA_SPIN_DEG);
        if (finished || turned > MAX_SPIN_DEG || now - wfStateStart > INSIDE_TIMEOUT_MS) {
            enterWallState(WF_FOLLOW);
        } else {
            spinLeft(CORNER_SPIN_MMS);          // the speed controller supplies the torque it takes
            return;
        }
    }

    if (wfState == WF_OUTSIDE_FWD) {
        // Wall ended: keep going straight until the wheel axle is one standoff past the corner
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        if (travelledMm() >= outsideFwdMm || now - wfStateStart > OUTSIDE_FWD_TIMEOUT_MS) {
            enterWallState(WF_OUTSIDE_TURN);
        } else {
            driveStraight(speed, +1);
            return;
        }
    }

    if (wfState == WF_OUTSIDE_TURN) {
        // Spin right in place (same proven mechanism as the inside corner, measured by the encoders)
        float turned = turnedDeg();
        if (turned >= OUTSIDE_TURN_DEG || now - wfStateStart > OUTSIDE_TIMEOUT_MS) {
            Serial.print("outside turn done, deg="); Serial.println(turned);
            enterWallState(WF_OUTSIDE_FIND);
        } else {
            spinRight(CORNER_SPIN_MMS);
            return;
        }
    }

    if (wfState == WF_OUTSIDE_FIND) {
        // Now facing along the new wall: drive straight until the right sensor sees it (twice in a row)
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        bool wallNear = (d > 0 && d < wallTarget + WALL_FOUND_CM);
        if (wallNear) wfFoundCount++; else wfFoundCount = 0;
        if (wfFoundCount >= 2 || travelledMm() > OUTSIDE_FIND_MAX_MM || now - wfStateStart > OUTSIDE_TIMEOUT_MS) {
            enterWallState(WF_FOLLOW);
        } else {
            driveStraight(speed, +1);
            return;
        }
    }

    // ---------------- WF_FOLLOW ----------------

    // 1. Inside corner: wall suddenly ahead
    if (frontBlocked) {
        enterWallState(WF_INSIDE_TURN);
        spinLeft(CORNER_SPIN_MMS);
        return;
    }

    // Slow down smoothly as a wall approaches: full speed at slowCm, down to V_MIN_MMS at the stop distance
    float driveSpeed = speed;
    if (front > 0 && front < slowCm) {
        float frac = constrain((front - stopCm) / (slowCm - stopCm), 0.0f, 1.0f);
        driveSpeed = V_MIN_MMS + (speed - V_MIN_MMS) * frac;
    }

    // 2. Capture the target: drive straight (encoder-balanced) and average the first valid readings
    if (wallTarget < 0) {
        if (d > 0) {
            wfCalSum += d;
            wfCalCount++;
            if (wfCalCount >= CAL_SAMPLES) {
                wallTarget = constrain(wfCalSum / wfCalCount, TARGET_MIN_CM, TARGET_MAX_CM);
                wfLastGood = wallTarget;
                Serial.print("target="); Serial.println(wallTarget);
            }
        }
        driveStraight(speed, +1);
        return;
    }

    // 3. Outside (convex) corner: the right sensor loses the wall.
    //    Lost = no echo OR a reading well beyond the target. If it stays lost for OUTSIDE_LOST_MM of travel
    //    (or jumps a long way in one step) the wall has ended -> drive forward, then arc around it.
    if (d < 0) {
        wfMisses++;
    } else {
        wfMisses = 0;
    }
    bool sharpJump = (d > 0 && wfLastGood > 0 && (d - wfLastGood) > OUTSIDE_JUMP_CM);
    bool lostReading = (d < 0) || (d > wallTarget + OUTSIDE_FAR_CM);
    if (lostReading) {
        wfNearStreak = 0;
        if (!wfLost) { wfLost = true; wfLostAtMm = odoDist; }
    } else {
        if (wfNearStreak < 100) wfNearStreak++;
        if (wfNearStreak >= 2) wfLost = false; // two good readings in a row: the wall is really still there (one stray echo doesn't count)
    }
    bool wallGone = (wfLost && fabs(odoDist - wfLostAtMm) >= OUTSIDE_LOST_MM)
                    || (d > 0 && d > wallTarget + OUTSIDE_LOST_CM)
                    || (wfMisses > MAX_MISSES);
    if (sharpJump || wallGone) {
        // Drive on until the axle is one standoff (wallTarget) past the corner, minus what we've already covered since losing it
        float lostTravel = wfLost ? fabs(odoDist - wfLostAtMm) : 0;
        outsideFwdMm = fmaxf(OUTSIDE_FWD_MIN_MM, wallTarget * 10.0f + OUTSIDE_FWD_OFFSET_MM - lostTravel);
        Serial.print("outside corner: driving on mm="); Serial.println(outsideFwdMm);
        enterWallState(WF_OUTSIDE_FWD);
        driveStraight(speed, +1);
        return;
    }

    // 4. Normal PD wall following (also handles gradual corners)
    float error;
    if (d < 0) {
        error = (wfLastGood > 0) ? (wfLastGood - wallTarget) : 0;   // brief dropout: hold last good value (0 if none yet)
    } else {
        wfFiltered = (wfFiltered < 0) ? d : 0.6 * wfFiltered + 0.4 * d;
        wfLastGood = wfFiltered;
        error = wfFiltered - wallTarget;           // + too far, - too close

        // Wall slope from the encoders: how much the wall distance changed per mm actually driven.
        // (cm -> mm factor of 10 makes it dimensionless, roughly tan of the angle to the wall.)
        if (wfPrevD < 0) {
            wfPrevD = wfFiltered;
            wfPrevDist = odoDist;
        } else {
            float ds = fabs(odoDist - wfPrevDist);
            if (ds >= WALL_MIN_DS_MM) {
                float slope = (wfFiltered - wfPrevD) * 10.0 / ds;
                wfRate = 0.7 * wfRate + 0.3 * slope;
                wfPrevD = wfFiltered;
                wfPrevDist = odoDist;
            }
        }
    }

    // Right after a corner the robot is rarely parallel to the wall and the reading is off.
    // Cap steering for a short DISTANCE so it eases back instead of pivoting hard (the "diagonal" swing).
    bool settling = (wfSettleMm > 0 && travelledMm() < wfSettleMm);
    float capAway   = settling ? fminf(MAX_CORR, SETTLE_MAX_CORR)        : MAX_CORR;
    float capToward = settling ? fminf(MAX_CORR_TOWARD, SETTLE_MAX_CORR) : MAX_CORR_TOWARD;
    float corr = constrain(WALL_KP * error + WALL_KD * wfRate, -capAway, capToward);   // mm/s of wheel-speed difference

    // Steer by speeding up the outer wheel and slowing the inner one. The inner wheel can't go
    // below V_MIN_MMS (it can't hold a lower speed), so whatever it can't give is added to the outer wheel.
    float mag  = fabsf(corr);
    float slow = driveSpeed - mag;
    float fast = driveSpeed + mag * 0.5f;
    if (slow < V_MIN_MMS) {
        fast += (V_MIN_MMS - slow);
        slow = V_MIN_MMS;
    }
    fast = fminf(fast, V_MAX_MMS);

    float leftSpeed, rightSpeed;
    if (corr < 0) {                        // too close: steer away (left)
        leftSpeed  = slow;
        rightSpeed = fast;
    } else {                               // too far: steer toward (right)
        leftSpeed  = fast;
        rightSpeed = slow;
    }
    setWheelSpeeds(leftSpeed, rightSpeed);

    Serial.print("front="); Serial.print(front);
    Serial.print(" right="); Serial.print(d);
    Serial.print(" target="); Serial.print(wallTarget);
    Serial.print(" lostMm="); Serial.print(wfLost ? fabs(odoDist - wfLostAtMm) : 0);
    Serial.print(" slope="); Serial.print(wfRate);
    Serial.print(" v="); Serial.print(vNow);
    Serial.print(" miss="); Serial.println(wfMisses);
}

// ---------------------------------------------------------------------
// Three-sensor line following for gradual bends, 90-degree corners and sharp (acute) corners.
// ---------------------------------------------------------------------
// This robot's adjustable IR modules output HIGH over white tape.
const bool LINE_SENSOR_ACTIVE_LOW = true;
const float LINE_GENTLE_TURN_FACTOR = 0.30f;  // inner wheel fraction on gradual bends

// Sharp corners. The robot remembers its HEADING (from the encoders) while it is centred on the tape. When it has to
// spin to find the line again, the line it just came along is directly BEHIND it (about 180 degrees from that heading),
// so tape found more than LINE_REVERSE_DEG away from the remembered heading is treated as "the way I came" and ignored.
// A corner sharper than 90 degrees means a turn of more than 90 degrees, so this must be well above 90: a 30 degree
// corner is a 150 degree turn. Keep it a bit under 180 so the line behind (which can look ~10-20 degrees wide) is rejected.
const float LINE_REVERSE_DEG = 150.0f;                       // TUNE
// If a one-direction spin (started because a single side sensor saw tape) reaches LINE_REVERSE_DEG without finding
// the line, it was the wrong way: spin the other way once. If that also fails, stop.
//
// If ALL sensors lose the tape, the robot instead SWEEPS: it turns a little to one side, then back through the middle
// to the same angle on the other side, then a bigger angle each time (LINE_SWEEP_START_DEG, then x LINE_SWEEP_GROWTH,
// ...), up to LINE_REVERSE_DEG on both sides, always going LEFT first. It resumes following as soon as the middle sensor sees tape.
// Example with 20 deg and x1.7: left 20, right 20, left 34, right 34, left 58, right 58, left 98, right 98, left 150, right 150.
const float LINE_SWEEP_START_DEG = 20.0f;                    // TUNE: first (smallest) sweep angle each side
const float LINE_SWEEP_GROWTH    = 1.7f;                     // TUNE: each pair of legs is this much wider than the last
// The robot must not creep forward while it searches. (1) A sweep first brakes to a standstill (it waits until both wheels
// are slower than LINE_SETTLE_V_MMS, at most LINE_SETTLE_MAX_MS) before the first turn, and (2) while spinning, the
// encoders watch how far the robot's centre has moved from where the spin began and push both wheels back to cancel it.
const float LINE_SETTLE_V_MMS = 15.0f;
const unsigned long LINE_SETTLE_MAX_MS = 400;
// Wheel speed while SWEEPING (mm/s per wheel; the robot's turning rate is 2 x this / TRACK_MM). Slower than CORNER_SPIN_MMS so
// the sensors have time to catch the tape. It can't go below V_MIN_MMS (the slowest speed the wheels can hold), and the closer
// it is to V_MIN_MMS the less room the drift correction below has, so keep it at least ~20 above.
const float LINE_SWEEP_SPIN_MMS = 100.0f;                    // TUNE (CORNER_SPIN_MMS is 120)
const float LINE_DRIFT_KP  = 3.0f;                           // mm/s of correction (both wheels) per mm of drift
const float LINE_DRIFT_MAX = 30.0f;                          // cap, mm/s; also limited to (spin speed - V_MIN_MMS)
// Safety net for either kind of search (a full sweep out to LINE_REVERSE_DEG both ways is roughly 900 degrees of turning, ~11 s at the default sweep speed).
const unsigned long LINE_SEARCH_TIMEOUT_MS = 16000;

int lineLastSide = 0;                   // -1 = bend/search left, +1 = right
unsigned long lineLastRun = 0;
unsigned long lineLastPrint = 0;
unsigned long lineSearchStart = 0;
bool lineSearchStopped = false;
float lineHeading = 0;                  // odoTheta (radians) the last time the middle sensor was on the tape
bool  lineSpinning = false;             // currently spinning in place to (re)find the line
int   lineSpinDir = 0;                  // -1 = spinning left, +1 = spinning right
bool  lineSpinFlipped = false;          // already reversed the spin direction once
bool  lineSweeping = false;             // the spin is a widening left/right sweep (not one fixed direction)
int   lineSweepLeg = 0;                 // 0 = first leg; legs 2k and 2k+1 share the same angle (one each side)
bool  lineSettling = false;             // sweep is braking to a standstill before it starts turning
float lineSpinStartMm = 0;              // odoDist where the spin began (to cancel forward/backward drift)

bool lineSensorOnTape(int pin) {
    int value = digitalRead(pin);
    return LINE_SENSOR_ACTIVE_LOW ? (value == LOW) : (value == HIGH);
}

void resetLineFollow() {
    lineLastSide = 0;
    lineLastRun = 0;
    lineLastPrint = 0;
    lineSearchStart = 0;
    lineSearchStopped = false;
    lineHeading = odoTheta;
    lineSpinning = false;
    lineSpinDir = 0;
    lineSpinFlipped = false;
    lineSweeping = false;
    lineSweepLeg = 0;
    lineSettling = false;
}

/** Spin in place (dir -1 = left, +1 = right), holding the robot's centre where it is: any drift measured by the
 *  encoders is cancelled by adding the same correction to both wheels. */
void lineSpinInPlace(int dir, float v) {
    v = fmaxf(v, V_MIN_MMS);                                                     // the wheels can't hold less than this
    float maxTrim = fminf(LINE_DRIFT_MAX, v - V_MIN_MMS);                        // keep both wheels at or above V_MIN_MMS
    float trim = constrain(-LINE_DRIFT_KP * (odoDist - lineSpinStartMm), -maxTrim, maxTrim);
    if (dir < 0) setWheelSpeeds(-v + trim,  v + trim);    // spin left:  left wheel back, right wheel forward
    else         setWheelSpeeds( v + trim, -v + trim);    // spin right: left wheel forward, right wheel back
}

/** Sweep angle (degrees each side of the remembered heading) for a given leg. */
float lineSweepAmp(int leg) {
    return fminf(LINE_SWEEP_START_DEG * powf(LINE_SWEEP_GROWTH, (float)(leg / 2)), LINE_REVERSE_DEG);
}

/** Last leg of a sweep: the pair of legs at which the angle first reaches LINE_REVERSE_DEG. */
int lineSweepLastLeg() {
    int pair = 0;
    while (LINE_SWEEP_START_DEG * powf(LINE_SWEEP_GROWTH, (float)pair) < LINE_REVERSE_DEG) pair++;
    return 2 * pair + 1;
}

/** Spin in place in direction dir (-1 = left, +1 = right) until the line is found again. */
void lineStartSpin(int dir, unsigned long now) {
    lineSpinning = true;
    lineSpinDir = dir;
    lineSpinFlipped = false;
    lineSweeping = false;
    lineSettling = false;
    lineSpinStartMm = odoDist;
    lineSearchStart = now;
    lineLastSide = dir;
}

/** Start a widening left/right sweep, first toward dir (-1 = left, +1 = right). Each angle is done left then right if dir = -1. */
void lineStartSweep(int dir, unsigned long now) {
    lineSpinning = true;
    lineSpinDir = dir;
    lineSpinFlipped = false;
    lineSweeping = true;
    lineSweepLeg = 0;
    lineSettling = true;                 // brake first; the turn starts once the robot has stopped
    lineSearchStart = now;
    stopRobot();
}

/** Follow white tape. Side+middle patterns steer gradual bends; a lone side sensor starts an in-place spin toward it,
 *  and losing the tape completely starts a widening left/right sweep. Both are tracked with the encoders so they can't
 *  latch onto the line behind the robot, which is what used to happen at corners sharper than 90 degrees. */
void lineFollow(float speed) {
    unsigned long now = millis();
    if (now - lineLastRun < 30) return;
    lineLastRun = now;

    bool leftOn = lineSensorOnTape(in5);
    bool midOn  = lineSensorOnTape(in6);
    bool rightOn = lineSensorOnTape(in7);

    float base = constrain(speed, V_MIN_MMS, V_MAX_MMS);
    float inside = fmaxf(V_MIN_MMS, base * LINE_GENTLE_TURN_FACTOR);
    bool anyOn = leftOn || midOn || rightOn;

    // Degrees turned since the robot was last centred on the line. + = turned left.
    float dev = (odoTheta - lineHeading) * 57.2958f;

    // ---------------- Spinning to find the line ----------------
    if (lineSpinning) {
        if (midOn && fabsf(dev) < LINE_REVERSE_DEG) {
            // The middle sensor is on tape and it is NOT the line behind us: that's the new line. Resume following.
            lineSpinning = false;
            lineSweeping = false;
            lineSearchStart = 0;
            lineSearchStopped = false;
            if (fabsf(dev) > 20.0f) lineLastSide = (dev > 0) ? -1 : +1;   // remember which way the line went
            Serial.print("line found after turning deg="); Serial.println(dev);
            // fall through to the normal following below
        } else {
            if (lineSettling) {
                bool stopped = fabsf(vRight) < LINE_SETTLE_V_MMS && fabsf(vLeft) < LINE_SETTLE_V_MMS;
                if (stopped || now - lineSearchStart >= LINE_SETTLE_MAX_MS) {
                    lineSettling = false;
                    lineSpinStartMm = odoDist;                            // measure drift from where we actually stopped
                } else {
                    stopRobot();
                    return;
                }
            }
            float turnedAway = (lineSpinDir < 0) ? dev : -dev;            // degrees turned in the spin direction
            if (lineSweeping) {
                // Sweep: turn until this leg's angle is reached, then head for the other side with a bigger angle.
                float amp = lineSweepAmp(lineSweepLeg);
                bool reached = (lineSpinDir < 0) ? (dev >= amp) : (dev <= -amp);
                if (reached) {
                    lineSweepLeg++;
                    if (lineSweepLeg > lineSweepLastLeg()) {
                        lineSpinning = false;
                        lineSweeping = false;
                        lineSearchStopped = true;
                        stopRobot();
                        Serial.println("Line not found after full sweep; stopped");
                        return;
                    }
                    lineSpinDir = -lineSpinDir;
                    Serial.print("line sweep: now "); Serial.print(lineSpinDir < 0 ? "left" : "right");
                    Serial.print(" to "); Serial.println(lineSweepAmp(lineSweepLeg));
                }
            } else if (turnedAway >= LINE_REVERSE_DEG) {
                // We've swung all the way round to where we came from: this direction was wrong.
                if (!lineSpinFlipped) {
                    lineSpinFlipped = true;
                    lineSpinDir = -lineSpinDir;
                    Serial.println("line: wrong way, reversing the search");
                } else {
                    lineSpinning = false;
                    lineSearchStopped = true;
                    stopRobot();
                    Serial.println("Line not found; stopped");
                    return;
                }
            }
            if (now - lineSearchStart >= LINE_SEARCH_TIMEOUT_MS) {
                lineSpinning = false;
                lineSweeping = false;
                lineSearchStopped = true;
                stopRobot();
                Serial.println("Line not reacquired; stopped");
                return;
            }
            lineSpinInPlace(lineSpinDir, lineSweeping ? LINE_SWEEP_SPIN_MMS : CORNER_SPIN_MMS);
            return;
        }
    }

    // ---------------- Normal following ----------------
    if (anyOn) {
        if (lineSearchStopped) lineHeading = odoTheta;    // restarted after a failed search: this is the new "forward"
        lineSearchStart = 0;
        lineSearchStopped = false;

        if (leftOn && !midOn && !rightOn) {
            // The tape is far to the left: pivot left to negotiate a sharp corner.
            lineStartSpin(-1, now);
            lineSpinInPlace(-1, CORNER_SPIN_MMS);
        } else if (rightOn && !midOn && !leftOn) {
            // The tape is far to the right: pivot right to negotiate a sharp corner.
            lineStartSpin(+1, now);
            lineSpinInPlace(+1, CORNER_SPIN_MMS);
        } else if (leftOn && midOn && !rightOn) {
            // Gradual left bend: arc left while continuing forward.
            lineLastSide = -1;
            lineHeading = odoTheta;
            setWheelSpeeds(inside, base);
        } else if (rightOn && midOn && !leftOn) {
            // Gradual right bend: arc right while continuing forward.
            lineLastSide = +1;
            lineHeading = odoTheta;
            setWheelSpeeds(base, inside);
        } else if (midOn) {
            // Centered on the tape (including broad tape): proceed straight.
            lineHeading = odoTheta;
            setWheelSpeeds(base, base);
        } else if (leftOn && rightOn) {
            // Split/ambiguous pattern: retain the last bend direction if known.
            if (lineLastSide < 0) setWheelSpeeds(inside, base);
            else if (lineLastSide > 0) setWheelSpeeds(base, inside);
            else setWheelSpeeds(base, base);
        }
    } else {
        // All sensors lost the tape: a sharp corner (we've driven past the point where the line turns) or the end of
        // the line. Sweep left and right with a growing angle, ALWAYS starting with the left side.
        // Only tape that is not the line behind us is accepted (see LINE_REVERSE_DEG).
        if (lineSearchStopped) {
            stopRobot();
        } else {
            lineStartSweep(-1, now);          // -1 = left first; brakes to a stop, then turns in place
        }
    }

    if (now - lineLastPrint >= 200) {
        lineLastPrint = now;
        Serial.print("IR L/M/R=");
        Serial.print(leftOn); Serial.print('/');
        Serial.print(midOn); Serial.print('/');
        Serial.print(rightOn);
        Serial.print(" dev="); Serial.println(dev);
    }
}
