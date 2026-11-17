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
 * Motor A = RIGHT wheel, Motor B = LEFT wheel.
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

float TARGET_CM = 15.0;
float KP = 6.0;
int MAX_CORR = 150;
unsigned long FOLLOW_INTERVAL_MS = 60;

/** @brief Distance in cm from ultrasonic sensor, or -1 on no echo*/

float readDistanceCm(int trig, int echo) {
    digitalWrite(trig, LOW);    delayMicroseconds(2);
    digitalWrite(trig, HIGH);   delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, 25000);  // 25 ms timeout
    if (dur == 0) return -1;
    return dur * 0.0343 / 2.0;      // return distance * speed of sound in cm/micros / 2, for one way distance
}

float FOLLOW_DIST_CM = 15.0;
float DEADBAND_CM = 3.0;
float MAX_RANGE_CM = 100.0;
int   MIN_PWM = 80;

/** @brief Follow what'sin front of the robot with P-control, call often from loop()*/
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

    float error = filtered - FOLLOW_DIST_CM;   // + = target too far, - = too close
    Serial.print("front="); Serial.println(filtered);

    if (abs(error) < DEADBAND_CM) {
        stopRobot();
        return;
    }

    int power = (int)(KP * error);
    power = constrain(power, -maxSpeed, maxSpeed);
    if (power > 0) power = max(power, MIN_PWM);
    if (power < 0) power = min(power, -MIN_PWM);

    // same signed speed on both wheels = straight line
    driveSigned(in1, in2, enA, power);
    driveSigned(in3, in4, enB, power);
}

/** @brief Follow a wall on the robot's RIGHT with P-control, call often from loop()*/
void straightFollow(int speed) {
    static unsigned long lastRun = 0;
    static float filtered = -1;

    if (millis() - lastRun < FOLLOW_INTERVAL_MS) return;
    lastRun = millis();

    float d  = readDistanceCm(trigR, echoR);
    if (d < 0) {
        driveSigned(in1, in2, enA, speed);
        driveSigned(in3, in4, enB, speed - MAX_CORR);
        return;
    }

    filtered = (filtered < 0) ? d : 0.6 * filtered + 0.4 * d;   // smoothing for sensor noise

    float error = filtered - TARGET_CM; // pos - too far from wall, neg - too close
    int corr = constrain((int)(KP * error), -MAX_CORR, MAX_CORR);

    // slow ONE wheel only to steer at high base speed
    // too far -> slow right wheel (steer toward wall)
    // too close -> slow the left wheel (steer away)
    int leftSpeed = speed - max(-corr, 0);
    int rightSpeed = speed - max(corr, 0);

    driveSigned(in1, in2, enA, rightSpeed);
    driveSigned(in3, in4, enB, leftSpeed);
}



