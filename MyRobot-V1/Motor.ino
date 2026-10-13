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
 * @brief Turn left while driving forward: left wheel slower than right.
 * @param speed Speed of the outside (right) wheel, 0-255
 * @param turn  How much to slow the inside (left) wheel. Bigger = sharper.
 */
void turnLeft(int speed, int turn) {
    driveSigned(in1, in2, enA, speed - turn);  // left (inside) slower
    driveSigned(in3, in4, enB, speed);         // right (outside)
}

/** @brief Turn right while driving forward: right wheel slower than left. */
void turnRight(int speed, int turn) {
    driveSigned(in1, in2, enA, speed);         // left (outside)
    driveSigned(in3, in4, enB, speed - turn);  // right (inside) slower
}

/** @brief Turn left while backing up. */
void turnLeftBackward(int speed, int turn) {
    driveSigned(in1, in2, enA, -(speed - turn));
    driveSigned(in3, in4, enB, -speed);
}

/** @brief Turn right while backing up. */
void turnRightBackward(int speed, int turn) {
    driveSigned(in1, in2, enA, -speed);
    driveSigned(in3, in4, enB, -(speed - turn));
}

/** @brief Spin in place to the left: left wheel back, right wheel forward. */
void spinLeft(int speed) {
    reverse(in1, in2, enA, speed);
    drive(in3, in4, enB, speed);
}

/** @brief Spin in place to the right: left wheel forward, right wheel back. */
void spinRight(int speed) {
    drive(in1, in2, enA, speed);
    reverse(in3, in4, enB, speed);
}
