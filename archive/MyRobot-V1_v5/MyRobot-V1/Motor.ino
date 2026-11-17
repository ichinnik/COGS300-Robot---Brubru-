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
 * MIN_PWM (defined in MyRobot-V1.ino) is the lowest PWM that actually moves the robot.
 * Anything weaker only makes the motors hum, so driveSigned() treats it as "stop".
 *
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
// High-level: both motors (uses globals from MyRobot-V1.ino)
// ---------------------------------------------------------------------

/** @brief Stop both motors. */
void stopRobot() {
    stop(in1, in2, enA);
    stop(in3, in4, enB);
}

/** @brief Drive both wheels forward. speed: 0-255 */
void driveForward(int speed) {
    drive(in1, in2, enA, speed);
    drive(in3, in4, enB, speed);
}

/** @brief Drive both wheels backward. speed: 0-255 */
void driveBackward(int speed) {
    reverse(in1, in2, enA, speed);
    reverse(in3, in4, enB, speed);
}

/**
 * @brief Turn right while driving forward: right wheel slower than left.
 * @param speed Speed of the outside (left) wheel, 0-255
 * @param turn  How much to slow the inside (right) wheel. Bigger = sharper.
 *              If speed - turn is below MIN_PWM the inside wheel stops (robot pivots on it).
 */
void turnRight(int speed, int turn) {
    driveSigned(in1, in2, enA, speed - turn);  // right (inside) slower
    driveSigned(in3, in4, enB, speed);         // left (outside)
}

/** @brief Turn left while driving forward: left wheel slower than right. */
void turnLeft(int speed, int turn) {
    driveSigned(in1, in2, enA, speed);         // right (outside)
    driveSigned(in3, in4, enB, speed - turn);  // left (inside) slower
}

/** @brief Turn right while backing up. */
void turnRightBackward(int speed, int turn) {
    driveSigned(in1, in2, enA, -(speed - turn));
    driveSigned(in3, in4, enB, -speed);
}

/** @brief Turn left while backing up. */
void turnLeftBackward(int speed, int turn) {
    driveSigned(in1, in2, enA, -speed);
    driveSigned(in3, in4, enB, -(speed - turn));
}

/** @brief Spin in place to the right: right wheel back, left wheel forward. */
void spinRight(int speed) {
    reverse(in1, in2, enA, speed);
    drive(in3, in4, enB, speed);
}

/** @brief Spin in place to the left: right wheel forward, left wheel back. */
void spinLeft(int speed) {
    drive(in1, in2, enA, speed);
    reverse(in3, in4, enB, speed);
}

// ---------------------------------------------------------------------
// Sensing + autonomous modes
// ---------------------------------------------------------------------
unsigned long FOLLOW_INTERVAL_MS = 60;

// Follow-me
float FOLLOW_DIST_CM  = 15.0;
float DEADBAND_CM     = 3.0;
float MAX_RANGE_CM    = 100.0;
float FOLLOW_KP       = 2.0;    // extra PWM per cm of error BEYOND the deadband, added on top of MIN_PWM
                                // (power runs from MIN_PWM at the deadband edge up to the commanded speed)

// Wall following (base speed >= MIN_PWM)
float WALL_KP        = 3.0;
float WALL_KD        = 6.0;     // acts on the wall SLOPE (cm of wall-distance change per cm travelled x10, ~tan of the
                                // angle to the wall), measured from the encoders. Speed-independent, unlike the old per-second rate.
                                // 6.0 ~ the old 0.4 at ~150 mm/s. Retune once you know your real speed.
int   MAX_CORR       = 55;      // max steering AWAY from the wall. Slow wheel is clamped at MIN_PWM and the extra
                                // differential comes from speeding the fast wheel up, so keep speed + 1.5*MAX_CORR <= 255.
int   MAX_CORR_TOWARD = 55;     // max steering TOWARD the wall (convex curves need more authority)
float FRONT_STOP_CM  = 20.0;
int   MAX_MISSES     = 8;
float WALL_MIN_DS_MM = 8.0;     // need at least this much travel between slope samples (a tick is only a few mm)

// Corner handling
float FRONT_CLEAR_CM     = 25.0;  // inside corner: front must read > this to stop turning (hysteresis vs FRONT_STOP_CM)
float FRONT_SLOW_CM      = 40.0;  // start slowing down when a wall is this close ahead (can't go below MIN_PWM)
float WALL_REACQUIRE_CM  = 15.0;  // inside corner: right sensor must read < target + this to resume following
float OUTSIDE_JUMP_CM    = 25.0;  // reading jumps this far in ONE step vs. the filtered value = wall just ended (sharp outside corner)
float OUTSIDE_LOST_CM    = 60.0;  // reading this far beyond target = wall is gone (gradual loss); also lost after MAX_MISSES no-echoes
float OUTSIDE_FAR_CM     = 12.0;  // right reading this far beyond target counts as "lost" (no echo also counts as lost)
float ARC_FOUND_CM       = 10.0;  // arc ends when right sensor reads < target + this, twice in a row

// --- Encoder-based distances / angles (these replaced the *_MS timers; all are starting guesses, TUNE) ---
float OUTSIDE_LOST_MM    = 40.0;  // right sensor lost the wall for this much TRAVEL = wall has ended      (was OUTSIDE_LOST_MS 250)
float OUTSIDE_FWD_MM     = 100.0; // drive straight this far after losing the wall so the body clears it  (was OUTSIDE_FWD_MS 500)
float MIN_ARC_DEG        = 40.0;  // arc at least this many degrees before looking for the new wall       (was MIN_ARC_MS 500)
float MAX_ARC_DEG        = 135.0; // give up the arc after this many degrees
float MIN_SPIN_DEG       = 30.0;  // always spin at least this many degrees (ignores a noisy early 'clear') (was MIN_SPIN_MS 250)
float EXTRA_SPIN_DEG     = 6.0;   // keep spinning this many degrees AFTER the corner looks clear           (was EXTRA_SPIN_MS 80)
float MAX_SPIN_DEG       = 150.0; // give up the spin after this many degrees
float POST_TURN_SETTLE_MM = 150.0;// after any corner, limit steering for this much travel                 (was POST_TURN_SETTLE_MS 800)

// --- Safety-net timeouts (only matter if an encoder dies) ---
unsigned long INSIDE_TIMEOUT_MS      = 3000;  // give up spinning after this long
unsigned long OUTSIDE_TIMEOUT_MS     = 4000;  // give up arcing after this long
unsigned long OUTSIDE_FWD_TIMEOUT_MS = 2000;  // give up the straight run after this long

// --- PWM values (MIN_PWM = 100 is the lowest that moves the robot) ---
int   OUTSIDE_ARC_INNER_PWM = MIN_PWM;    // inner-wheel PWM during the outside-corner arc. Can't be weaker than MIN_PWM;
                                          // set to 0 for a tighter arc (inner wheel stops and the robot pivots on it)
int   CORNER_ARC_OUTER_PWM  = 200;        // outer-wheel PWM during the outside-corner arc (bigger gap to inner = tighter)
int   CORNER_SPIN_PWM       = 150;        // starting PWM for spinning in place at an inside corner. Spinning needs MORE torque
                                          // than driving straight. If the encoders show a stall, it is raised automatically.
int   SPIN_BOOST_STEP       = 15;         // PWM added each time a stall is detected during the spin
unsigned long STALL_MS      = 200;        // a wheel with no encoder ticks for this long while being driven = stalled
int   SETTLE_MAX_CORR       = 20;         // max steering (either way) during the post-corner settling distance

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
const int WF_FOLLOW = 0, WF_INSIDE_TURN = 1, WF_OUTSIDE_FWD = 2, WF_OUTSIDE_ARC = 3;
int wfState = WF_FOLLOW;
unsigned long wfStateStart = 0;
int   wfFoundCount = 0;          // consecutive 'new wall found' readings during the arc
bool  wfLost = false;            // right sensor currently doesn't see the wall
float wfLostAtMm = 0;            // odoDist when the wall was first lost
float wfSettleMm = 0;            // gentle-steering distance after a corner ends (0 = not settling)
float wfClearDeg = -1;           // degrees turned when the inside-corner turn first looked finished (-1 = not yet)
int   cornerSpinPwm = 150;       // current spin PWM (starts at CORNER_SPIN_PWM, raised on stall)

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
void followMe(int maxSpeed) {
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

    // MIN_PWM at the edge of the deadband, rising with distance error up to the commanded speed
    int top   = max(maxSpeed, MIN_PWM);
    int power = MIN_PWM + (int)(FOLLOW_KP * (fabs(error) - DEADBAND_CM));
    power = constrain(power, MIN_PWM, top);

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
    wfFoundCount = 0;
    wfClearDeg = -1;
    cornerSpinPwm = CORNER_SPIN_PWM;
    markMotion();
    resetStraight();                 // the target-capture run is driven straight
}

/** @brief Set wheel speeds by physical side. Signed: + forward, - backward.
 *  NOTE: per turnRight()/spinLeft(), motor A (enA) is the physical RIGHT wheel
 *  and motor B (enB) is the physical LEFT wheel. */
void setWheels(int leftSpeed, int rightSpeed) {
    driveSigned(in3, in4, enB, leftSpeed);
    driveSigned(in1, in2, enA, rightSpeed);
}

void enterWallState(int s) {
    wfState = s;
    wfStateStart = millis();
    markMotion();                    // turnedDeg() / travelledMm() now measure from here
    resetStall();                    // fresh stall timer for the new state
    wfClearDeg = -1;
    wfLost = false;
    wfFoundCount = 0;
    cornerSpinPwm = CORNER_SPIN_PWM;
    if (s == WF_OUTSIDE_FWD) resetStraight();
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
void straightFollow(int speed) {
    unsigned long now = millis();
    if (now - wfLastRun < FOLLOW_INTERVAL_MS) return;
    wfLastRun = now;

    float front = readDistanceCm(trigF, echoF);
    float d     = readDistanceCm(trigR, echoR);
    bool frontBlocked = (front > 0 && front < FRONT_STOP_CM);

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
            // Wheels not turning (encoders silent) -> not enough torque; add PWM
            if (wheelsStalled(STALL_MS)) {
                cornerSpinPwm = min(cornerSpinPwm + SPIN_BOOST_STEP, 255);
                resetStall();
            }
            spinLeft(max(speed, cornerSpinPwm));
            return;
        }
    }

    if (wfState == WF_OUTSIDE_FWD) {
        // Wall ended: keep going straight so the robot's body clears the corner
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        if (travelledMm() > OUTSIDE_FWD_MM || now - wfStateStart > OUTSIDE_FWD_TIMEOUT_MS) {
            enterWallState(WF_OUTSIDE_ARC);
        } else {
            driveStraight(speed, +1);
            return;
        }
    }

    if (wfState == WF_OUTSIDE_ARC) {
        // Arc right (toward where the wall went) until the right sensor finds the NEW wall.
        // Ignore the sensor until MIN_ARC_DEG has been turned, then require two near readings in a row.
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        float turned = turnedDeg();
        bool wallNear = (d > 0 && d < wallTarget + ARC_FOUND_CM);
        if (turned > MIN_ARC_DEG && wallNear) wfFoundCount++; else wfFoundCount = 0;
        if (wfFoundCount >= 2 || turned > MAX_ARC_DEG || now - wfStateStart > OUTSIDE_TIMEOUT_MS) {
            enterWallState(WF_FOLLOW);
        } else {
            setWheels(max(speed, CORNER_ARC_OUTER_PWM), OUTSIDE_ARC_INNER_PWM);   // left wheel fast, right wheel slow -> curve right
            return;
        }
    }

    // ---------------- WF_FOLLOW ----------------

    // 1. Inside corner: wall suddenly ahead
    if (frontBlocked) {
        enterWallState(WF_INSIDE_TURN);
        spinLeft(max(speed, cornerSpinPwm));
        return;
    }

    // Slow down as a wall approaches so there is time to react (can't drop below MIN_PWM,
    // so this only has an effect when the commanded speed is well above MIN_PWM)
    int driveSpeed = speed;
    if (front > 0 && front < FRONT_SLOW_CM) {
        driveSpeed = max(speed * 6 / 10, MIN_PWM);
        driveSpeed = min(driveSpeed, speed);
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
        if (!wfLost) { wfLost = true; wfLostAtMm = odoDist; }
    } else {
        wfLost = false;                        // a valid near reading: the wall is still there
    }
    bool wallGone = (wfLost && fabs(odoDist - wfLostAtMm) >= OUTSIDE_LOST_MM)
                    || (d > 0 && d > wallTarget + OUTSIDE_LOST_CM)
                    || (wfMisses > MAX_MISSES);
    if (sharpJump || wallGone) {
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
    int capAway   = settling ? min(MAX_CORR, SETTLE_MAX_CORR)        : MAX_CORR;
    int capToward = settling ? min(MAX_CORR_TOWARD, SETTLE_MAX_CORR) : MAX_CORR_TOWARD;
    int corr = constrain((int)(WALL_KP * error + WALL_KD * wfRate), -capAway, capToward);

    // Steer by speeding up the outer wheel and slowing the inner one. The inner wheel can't go
    // below MIN_PWM (it would just hum), so whatever it can't give is added to the outer wheel.
    int mag  = abs(corr);
    int slow = driveSpeed - mag;
    int fast = driveSpeed + mag / 2;
    if (slow < MIN_PWM) {
        fast += (MIN_PWM - slow);
        slow = MIN_PWM;
    }
    fast = min(fast, 255);

    int leftSpeed, rightSpeed;
    if (corr < 0) {                        // too close: steer away (left)
        leftSpeed  = slow;
        rightSpeed = fast;
    } else {                               // too far: steer toward (right)
        leftSpeed  = fast;
        rightSpeed = slow;
    }
    setWheels(leftSpeed, rightSpeed);

    Serial.print("front="); Serial.print(front);
    Serial.print(" right="); Serial.print(d);
    Serial.print(" target="); Serial.print(wallTarget);
    Serial.print(" lostMm="); Serial.print(wfLost ? fabs(odoDist - wfLostAtMm) : 0);
    Serial.print(" slope="); Serial.print(wfRate);
    Serial.print(" miss="); Serial.println(wfMisses);
}
