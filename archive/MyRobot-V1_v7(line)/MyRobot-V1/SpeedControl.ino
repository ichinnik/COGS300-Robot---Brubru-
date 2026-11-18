/**
 * @file SpeedControl.ino
 * @brief Closed-loop wheel speed control. Every speed in the sketch is in mm/s and is measured
 *        by the encoders; PWM is only the actuator signal this controller works out.
 *
 * How it works (one controller per wheel, run every ENC_INTERVAL_MS from updateEncoders()):
 *
 *   PWM = feedforward(target) + SPD_KP * error + integral
 *
 *   - feedforward: a straight line through two measured points, (V_MIN_MMS -> MIN_PWM) and
 *     (V_MAX_MMS -> 255). It gets the PWM roughly right immediately.
 *   - error: target speed minus the speed the encoder measures (vRight / vLeft).
 *   - integral: removes whatever the feedforward gets wrong (motor mismatch, battery sag, floor
 *     friction, the extra torque a spin needs). It keeps raising PWM until the wheel really
 *     turns at the target speed, which also replaces the old "stall -> add PWM" boost.
 *
 * Use setWheelSpeeds() from anywhere. Speeds are signed (+ forward, - backward). A wheel asked
 * for less than V_MIN_MMS (but not zero) is stopped, because the robot can't hold a speed that
 * low: it only hums at MIN_PWM.
 *
 * Calibrating V_MIN_MMS / V_MAX_MMS: see the comments next to them in MyRobot-V1.ino.
 * printSpeedTelemetry() shows target (tR/tL), measured (vR/vL) and the PWM used (pR/pL).
 */

// ---------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------
const float SPD_KP    = 0.20;    // PWM per (mm/s) of speed error
const float SPD_KI    = 1.00;    // PWM per (mm/s) of speed error, per second
const float SPD_I_MAX = 120.0;   // integrator limit, in PWM

// ---------------------------------------------------------------------
// State.  Index 0 = motor A (RIGHT wheel), index 1 = motor B (LEFT wheel).
// ---------------------------------------------------------------------
bool  spdActive    = false;      // false = controller off (stopRobot() was called)
float spdTarget[2] = {0, 0};     // wanted speed, mm/s, signed (0 = stopped)
float spdI[2]      = {0, 0};     // integrator, PWM
int   spdPwm[2]    = {0, 0};     // PWM last sent to each wheel, signed (also for telemetry)

/** PWM expected to give |mag| mm/s. The integrator corrects whatever this gets wrong. */
float feedforwardPwm(float mag) {
    float p = MIN_PWM + (mag - V_MIN_MMS) * (255.0f - MIN_PWM) / (V_MAX_MMS - V_MIN_MMS);
    return constrain(p, (float)MIN_PWM, 255.0f);
}

/** Send spdPwm[w] to the motor driver. */
void applyWheelPwm(int w) {
    if (w == 0) driveSigned(in1, in2, enA, spdPwm[0]);   // A = RIGHT
    else        driveSigned(in3, in4, enB, spdPwm[1]);   // B = LEFT
}

/**
 * @brief Set the wanted speed of each wheel.
 * @param leftMms  left wheel,  mm/s, + forward / - backward
 * @param rightMms right wheel, mm/s, + forward / - backward
 * Safe to call as often as you like (every loop pass is fine).
 */
void setWheelSpeeds(float leftMms, float rightMms) {
    float want[2] = { rightMms, leftMms };               // [0] = A = RIGHT, [1] = B = LEFT
    for (int w = 0; w < 2; w++) {
        float v = constrain(want[w], -V_MAX_MMS, V_MAX_MMS);
        if (fabsf(v) < V_MIN_MMS) v = 0;                 // slower than the robot can hold: stop this wheel

        bool sameWay = (v > 0 && spdTarget[w] > 0) || (v < 0 && spdTarget[w] < 0);
        if (!sameWay) spdI[w] = 0;                       // starting, stopping or reversing: forget the integrator
        spdTarget[w] = v;

        // Act at once (with the feedforward) instead of waiting for the next control tick
        int wantPwm = spdPwm[w];
        if (v == 0)         wantPwm = 0;
        else if (!sameWay)  wantPwm = ((v > 0) ? 1 : -1) * (int)feedforwardPwm(fabsf(v));
        if (wantPwm != spdPwm[w]) {
            spdPwm[w] = wantPwm;
            applyWheelPwm(w);
        }
    }
    spdActive = true;
}

/** Turn the controller off and clear its state. stopRobot() calls this, then cuts the motor pins. */
void stopWheelControl() {
    spdActive = false;
    for (int w = 0; w < 2; w++) {
        spdTarget[w] = 0;
        spdI[w] = 0;
        spdPwm[w] = 0;
    }
}

/** One control step. Called by updateEncoders() right after the speeds are measured. */
void runSpeedControl(float dt) {
    if (!spdActive) return;
    float meas[2] = { vRight, vLeft };

    for (int w = 0; w < 2; w++) {
        if (spdTarget[w] == 0) {
            spdI[w] = 0;
            if (spdPwm[w] != 0) { spdPwm[w] = 0; applyWheelPwm(w); }
            continue;
        }

        // Work in "direction of travel" terms so forward and backward use the same maths
        float s    = (spdTarget[w] > 0) ? 1.0f : -1.0f;
        float tgt  = fabsf(spdTarget[w]);
        float err  = tgt - s * meas[w];                  // + = too slow
        float base = feedforwardPwm(tgt) + SPD_KP * err;

        // Anti-windup: don't keep integrating while pinned at a PWM limit and the error pushes further into it
        bool atMax = (base + spdI[w] >= 255.0f && err > 0);
        bool atMin = (base + spdI[w] <= MIN_PWM && err < 0);
        if (!atMax && !atMin) {
            spdI[w] = constrain(spdI[w] + SPD_KI * err * dt, -SPD_I_MAX, SPD_I_MAX);
        }

        float mag = constrain(base + spdI[w], (float)MIN_PWM, 255.0f);   // never below what moves the robot
        int pwm = (int)(s * mag);
        if (pwm != spdPwm[w]) {
            spdPwm[w] = pwm;
            applyWheelPwm(w);
        }
    }
}

/** Telemetry for tuning / calibration (call at <= 5 Hz). */
void printSpeedTelemetry() {
    Serial.print("vR=");  Serial.print(vRight);
    Serial.print(" vL="); Serial.print(vLeft);
    Serial.print(" tR="); Serial.print(spdTarget[0]);
    Serial.print(" tL="); Serial.print(spdTarget[1]);
    Serial.print(" pR="); Serial.print(spdPwm[0]);
    Serial.print(" pL="); Serial.println(spdPwm[1]);
}
