/**
 * @file Motor.ino
 * @brief Simple H-bridge motor control helpers.
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
 * Motor A = LEFT wheel, Motor B = RIGHT wheel.
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
 *        Positive = forward, negative = backward, 0 = stop.
 *
 * @param speed Signed speed from -255 to 255
 */
void driveSigned(int in1, int in2, int enA, int speed) {
    if (speed > 0) {
        drive(in1, in2, enA, speed);
    } else if (speed < 0) {
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
float FOLLOW_KP       = 2.0;
int   FOLLOW_MIN_PWM  = 30;    // lowest PWM that reliably moves the robot

// Wall following (base speed ~60)
float WALL_KP        = 4.0;
float WALL_KD        = 0.4;
int   MAX_CORR       = 55;     // max steering AWAY from the wall (must stay <= speed - WALL_MIN_PWM)
int   MAX_CORR_TOWARD = 55;    // max steering TOWARD the wall (convex curves need more authority); inner wheel may drop to 0
int   WALL_MIN_PWM   = 30;
float FRONT_STOP_CM  = 15.0;
int   MAX_MISSES     = 8;

// Corner handling
float FRONT_CLEAR_CM     = 25.0;  // inside corner: front must read > this to stop turning (hysteresis vs FRONT_STOP_CM)
float FRONT_SLOW_CM      = 40.0;  // start slowing down when a wall is this close ahead
float WALL_REACQUIRE_CM  = 15.0;  // inside corner: right sensor must read < target + this to resume following
float OUTSIDE_JUMP_CM    = 25.0;  // reading jumps this far in ONE step vs. the filtered value = wall just ended (sharp outside corner)
float OUTSIDE_LOST_CM    = 60.0;  // reading this far beyond target = wall is gone (gradual loss); also lost after MAX_MISSES no-echoes
float OUTSIDE_FAR_CM     = 12.0;  // right reading this far beyond target counts as "lost" (no echo also counts as lost)
unsigned long OUTSIDE_LOST_MS = 250;     // right sensor lost the wall continuously for this long = wall has ended (time-based, so it does not depend on loop speed)
unsigned long OUTSIDE_FWD_MS     = 500;   // drive straight after losing wall so the robot body clears the corner  (TUNE!)
unsigned long INSIDE_TIMEOUT_MS  = 3000;  // give up spinning after this long
unsigned long OUTSIDE_TIMEOUT_MS = 4000;  // give up arcing after this long
unsigned long MIN_ARC_MS         = 500;   // always arc at least this long before looking for the new wall (stops the arc ending instantly)
float ARC_FOUND_CM               = 10.0;  // arc ends when right sensor reads < target + this, twice in a row
int   OUTSIDE_ARC_INNER_PWM = 30;         // inner-wheel PWM during the outside-corner arc (smaller = tighter)
int   CORNER_SPIN_PWM       = 110;        // PWM for spinning in place at an inside corner. Spinning needs MORE torque than
                                          // driving straight; ~60 often stalls the motors. Raise until it spins reliably.
int   CORNER_ARC_OUTER_PWM  = 100;        // outer-wheel PWM during the outside-corner arc
unsigned long EXTRA_SPIN_MS = 80;         // keep spinning this long AFTER the corner looks clear (bigger = turns more)
unsigned long POST_TURN_SETTLE_MS = 800;  // after any corner, limit steering for this long so it can straighten out gently
int   SETTLE_MAX_CORR       = 20;         // max steering (either way) during that settling period
unsigned long MIN_SPIN_MS   = 250;        // always spin at least this long (ignores a noisy early 'clear' reading)

// Captured wall target
float TARGET_MIN_CM  = 8.0;
float TARGET_MAX_CM  = 40.0;
const int CAL_SAMPLES = 5;

// Wall-follow state (global so it can be reset)
float wallTarget  = -1;        // -1 = not captured yet
float wfFiltered  = -1;
float wfRate      = 0;
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
unsigned long wfLostSince = 0;   // when the right sensor first lost the wall (0 = wall currently seen)
unsigned long wfSettleUntil = 0;  // gentle-steering window after a corner ends
unsigned long wfClearAt = 0;   // when the inside-corner turn first looked finished (0 = not yet)

/** @brief Distance in cm from ultrasonic sensor, or -1 on no echo */
float readDistanceCm(int trig, int echo) {
    digitalWrite(trig, LOW);    delayMicroseconds(2);
    digitalWrite(trig, HIGH);   delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, 25000);  // 25 ms timeout
    if (dur == 0) return -1;
    return dur * 0.0343 / 2.0;
}

/** @brief Follow whatever is in front of the robot. Call often from loop(). */
void followMe(int maxSpeed) {
    static unsigned long lastRun = 0;
    static float filtered = -1;

    if (millis() - lastRun < FOLLOW_INTERVAL_MS) return;
    lastRun = millis();

    float d = readDistanceCm(trigF, echoF);
    if (d < 0 || d > MAX_RANGE_CM) {      // nothing in range: stop and wait
        filtered = -1;
        stopRobot();
        return;
    }

    filtered = (filtered < 0) ? d : 0.5 * filtered + 0.5 * d;

    float error = filtered - FOLLOW_DIST_CM;   // + = too far, - = too close
    Serial.print("front="); Serial.println(filtered);

    if (abs(error) < DEADBAND_CM) {
        stopRobot();
        return;
    }

    int power = constrain((int)(FOLLOW_KP * error), -maxSpeed, maxSpeed);
    if (power > 0) power = max(power,  FOLLOW_MIN_PWM);
    if (power < 0) power = min(power, -FOLLOW_MIN_PWM);

    driveSigned(in1, in2, enA, power);
    driveSigned(in3, in4, enB, power);
}

/** @brief Call when wall following starts; the next readings set the target. */
void resetWallFollow() {
    wallTarget = -1;
    wfFiltered = -1;
    wfRate     = 0;
    wfLastGood = -1;
    wfMisses   = 0;
    wfCalCount = 0;
    wfCalSum   = 0;
    wfLastRun  = millis();
    wfState    = WF_FOLLOW;
    wfStateStart = millis();
    wfSettleUntil = 0;
    wfLostSince = 0;
    wfFoundCount = 0;
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
    wfClearAt = 0;
    wfLostSince = 0;
    wfFoundCount = 0;
    if (s == WF_FOLLOW) {
        wfSettleUntil = millis() + POST_TURN_SETTLE_MS;
        wfLastGood = -1;     // forget the OLD wall's distance, or the first reading on the new wall looks like a huge jump
    }
    // Reset the PD memory so we don't carry a stale derivative into FOLLOW
    wfFiltered = -1;
    wfRate = 0;
    wfMisses = 0;
    Serial.print("wallState="); Serial.println(s);
}

/** @brief Follow a wall on the robot's RIGHT, holding the distance it started at.
 *         Handles gradual corners (PD), inside corners and outside corners. */
void straightFollow(int speed) {
    unsigned long now = millis();
    if (now - wfLastRun < FOLLOW_INTERVAL_MS) return;
    float dt = (now - wfLastRun) / 1000.0;
    wfLastRun = now;

    float front = readDistanceCm(trigF, echoF);
    float d     = readDistanceCm(trigR, echoR);
    bool frontBlocked = (front > 0 && front < FRONT_STOP_CM);

    // ---------------- Corner states ----------------
    if (wfState == WF_INSIDE_TURN) {
        // Spin left (away from the right-hand wall) until front is clear AND
        // the right sensor sees the new wall at a sensible distance.
        bool frontClear = (front < 0 || front > FRONT_CLEAR_CM);
        bool wallSeen   = (d > 0 && d < wallTarget + WALL_REACQUIRE_CM);
        // Latch the moment the turn first looks done, then keep spinning EXTRA_SPIN_MS more
        if (wfClearAt == 0 && frontClear && wallSeen && now - wfStateStart > MIN_SPIN_MS) {
            wfClearAt = now;
        }
        bool finished = (wfClearAt != 0 && now - wfClearAt >= EXTRA_SPIN_MS);
        if (finished || now - wfStateStart > INSIDE_TIMEOUT_MS) {
            enterWallState(WF_FOLLOW);
        } else {
            spinLeft(max(speed, CORNER_SPIN_PWM));
            return;
        }
    }

    if (wfState == WF_OUTSIDE_FWD) {
        // Wall ended: keep going straight so the robot's body clears the corner
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        if (now - wfStateStart > OUTSIDE_FWD_MS) {
            enterWallState(WF_OUTSIDE_ARC);
        } else {
            setWheels(speed, speed);
            return;
        }
    }

    if (wfState == WF_OUTSIDE_ARC) {
        // Arc right (toward where the wall went) until the right sensor finds the NEW wall.
        // Ignore the sensor for MIN_ARC_MS, then require two near readings in a row.
        if (frontBlocked) { enterWallState(WF_INSIDE_TURN); return; }
        bool wallNear = (d > 0 && d < wallTarget + ARC_FOUND_CM);
        if (now - wfStateStart > MIN_ARC_MS && wallNear) wfFoundCount++; else wfFoundCount = 0;
        if (wfFoundCount >= 2 || now - wfStateStart > OUTSIDE_TIMEOUT_MS) {
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
        spinLeft(max(speed, CORNER_SPIN_PWM));
        return;
    }

    // Slow down as a wall approaches so there is time to react
    int driveSpeed = speed;
    if (front > 0 && front < FRONT_SLOW_CM) {
        driveSpeed = max(speed * 6 / 10, WALL_MIN_PWM + MAX_CORR);
        driveSpeed = min(driveSpeed, speed);
    }

    // 2. Capture the target: drive straight and average the first valid readings
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
        setWheels(speed, speed);
        return;
    }

    // 3. Outside (convex) corner: the right sensor loses the wall.
    //    Lost = no echo OR a reading well beyond the target. If it stays lost for OUTSIDE_LOST_MS
    //    (or jumps a long way in one step) the wall has ended -> drive forward, then arc around it.
    if (d < 0) {
        wfMisses++;
    } else {
        wfMisses = 0;
    }
    bool sharpJump = (d > 0 && wfLastGood > 0 && (d - wfLastGood) > OUTSIDE_JUMP_CM);
    bool lostReading = (d < 0) || (d > wallTarget + OUTSIDE_FAR_CM);
    if (lostReading) {
        if (wfLostSince == 0) wfLostSince = now;
    } else {
        wfLostSince = 0;                       // a valid near reading: the wall is still there
    }
    bool wallGone = (wfLostSince != 0 && now - wfLostSince >= OUTSIDE_LOST_MS)
                    || (d > 0 && d > wallTarget + OUTSIDE_LOST_CM)
                    || (wfMisses > MAX_MISSES);
    if (sharpJump || wallGone) {
        enterWallState(WF_OUTSIDE_FWD);
        setWheels(speed, speed);
        return;
    }

    // 4. Normal PD wall following (also handles gradual corners)
    float error;
    if (d < 0) {
        error = (wfLastGood > 0) ? (wfLastGood - wallTarget) : 0;   // brief dropout: hold last good value (0 if none yet)
    } else {
        float prev = wfFiltered;
        wfFiltered = (wfFiltered < 0) ? d : 0.6 * wfFiltered + 0.4 * d;
        wfLastGood = wfFiltered;
        error = wfFiltered - wallTarget;           // + too far, - too close

        if (prev >= 0 && dt > 0) {
            float newRate = (wfFiltered - prev) / dt;
            wfRate = 0.7 * wfRate + 0.3 * newRate;
        }
    }

    // Right after a corner the robot is rarely parallel to the wall and the reading is off.
    // Cap steering for a short while so it eases back instead of pivoting hard (the "diagonal" swing).
    bool settling = (now < wfSettleUntil);
    int capAway   = settling ? min(MAX_CORR, SETTLE_MAX_CORR)        : MAX_CORR;
    int capToward = settling ? min(MAX_CORR_TOWARD, SETTLE_MAX_CORR) : MAX_CORR_TOWARD;
    int corr = constrain((int)(WALL_KP * error + WALL_KD * wfRate), -capAway, capToward);

    int leftSpeed, rightSpeed;
    if (corr < 0) {                        // too close: steer away (left)
        leftSpeed  = driveSpeed + corr;
        rightSpeed = driveSpeed - corr / 2;
    } else {                               // too far: steer toward (right)
        rightSpeed = driveSpeed - corr;
        leftSpeed  = driveSpeed + corr / 2;
    }

    leftSpeed  = constrain(leftSpeed,  WALL_MIN_PWM, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    if (rightSpeed < WALL_MIN_PWM) rightSpeed = 0;   // below min PWM the wheel only hums; let it stop so the robot can pivot toward the wall

    // BUG FIX: the original computed these speeds but never applied them to the motors
    setWheels(leftSpeed, rightSpeed);

    Serial.print("front="); Serial.print(front);
    Serial.print(" right="); Serial.print(d);
    Serial.print(" target="); Serial.print(wallTarget);
    Serial.print(" lostMs="); Serial.print(wfLostSince ? (now - wfLostSince) : 0);
    Serial.print(" miss="); Serial.println(wfMisses);
}
