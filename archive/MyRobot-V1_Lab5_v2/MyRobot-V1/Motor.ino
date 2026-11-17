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
int   MAX_CORR       = 30;     // must stay <= speed - WALL_MIN_PWM
int   WALL_MIN_PWM   = 30;
float FRONT_STOP_CM  = 15.0;
int   MAX_MISSES     = 4;

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
}

/** @brief Follow a wall on the robot's RIGHT, holding the distance it started at. */
void straightFollow(int speed) {
    unsigned long now = millis();
    if (now - wfLastRun < FOLLOW_INTERVAL_MS) return;
    float dt = (now - wfLastRun) / 1000.0;
    wfLastRun = now;

    // 1. Wall ahead? Spin left (away from the right-hand wall)
    float front = readDistanceCm(trigF, echoF);
    if (front > 0 && front < FRONT_STOP_CM) {
        spinLeft(max(speed, WALL_MIN_PWM));
        return;
    }

    float d = readDistanceCm(trigR, echoR);

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
        driveSigned(in1, in2, enA, speed);
        driveSigned(in3, in4, enB, speed);
        return;
    }

    // 3. Normal wall following against wallTarget
    float error;

    if (d < 0) {
        wfMisses++;
        if (wfMisses > MAX_MISSES) {
            error = 0;
            wfRate = 0;
        } else {
            error = wfLastGood - wallTarget;
        }
    } else {
        wfMisses = 0;
        float prev = wfFiltered;
        wfFiltered = (wfFiltered < 0) ? d : 0.6 * wfFiltered + 0.4 * d;
        wfLastGood = wfFiltered;
        error = wfFiltered - wallTarget;   // + too far, - too close

        if (prev >= 0 && dt > 0) {
            float newRate = (wfFiltered - prev) / dt;
            wfRate = 0.7 * wfRate + 0.3 * newRate;
        }
    }

    int corr = constrain((int)(WALL_KP * error + WALL_KD * wfRate), -MAX_CORR, MAX_CORR);

    int leftSpeed, rightSpeed;
    if (corr < 0) {                        // too close: steer away
        leftSpeed  = speed + corr;
        rightSpeed = speed - corr / 2;
    } else {                               // too far: steer toward
        rightSpeed = speed - corr;
        leftSpeed  = speed + corr / 2;
    }

    leftSpeed  = constrain(leftSpeed,  WALL_MIN_PWM, 255);
    rightSpeed = constrain(rightSpeed, WALL_MIN_PWM, 255);

    Serial.print("front="); Serial.print(front);
    Serial.print(" right="); Serial.print(d);
    Serial.print(" target="); Serial.print(wallTarget);
}