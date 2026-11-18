// MyRobot-V1.ino
// This file must be named the same as your sketch folder
//
// Remote control over Serial. The Processing sketch (MyRobotControl.pde)
// sends lines like "W 200\n" = <command letter> <space> <speed in mm/s>.
//
//   W = forward            S = backward
//   A = spin left          D = spin right        (in place)
//   Q = forward + left     E = forward + right   (turn while driving)
//   Z = backward + left    C = backward + right
//   X = stop             L = IR line follow

int enA = 9;   // Enable pin for Motor A (RIGHT) - must be a PWM-capable pin         !!! NOTE!! CHANGE PINS ACCORDING TO OUR WIRING!!!
int in1 = 12;   // Direction control pin 1 for Motor A
int in2 = 13;   // Direction control pin 2 for Motor A
int enB = 10;   // Enable pin for Motor B (LEFT) - must be a PWM-capable pin
int in3 = 4;   // Direction control pin 3 for Motor B
int in4 = 5;   // Direction control pin 4 for Motor B
int trigR = A3; // Trigger pin for ultrasonic sensor R
int echoR = A4; // Echo pin for ultrasonic sensor R
int trigF = A0; // Trigger pin for ultrasonic sensor F
int echoF = A1; // Echo pin for ultrasonic sensor F
int followSpeed = 0;
int in5 = 8; // output pin for left IR sensor
int in6 = 6; // output pin for middle IR sensor
int in7 = 7; // output pin for right IR sensor

// ---- Speed limits. ALL speeds in the sketch are mm/s, measured by the wheel encoders. ----
const int   MIN_PWM   = 30;      // lowest PWM that moves the robot. Only the speed controller (SpeedControl.ino) uses PWM.
const float V_MIN_MMS = 80.0;    // slowest speed the robot can hold = its real speed at MIN_PWM.            TUNE (see notes)
const float V_MAX_MMS = 400.0;   // fastest speed it can hold with some headroom, even on a tired battery.   TUNE (see notes)
// How to measure them: watch "vR= vL= tR= tL= pR= pL=" in the serial output.
//  V_MIN_MMS: set it low (e.g. 40) and drive "W 40". The controller can't go below MIN_PWM, so pR/pL sit at 60 and
//             vR/vL show the real minimum speed. Put that number here.
//  V_MAX_MMS: set it high (e.g. 600) and drive "W 600". If pR/pL pin at 255 the robot can't reach the target; the vR/vL it
//             settles at is its top speed. Use ~85-90% of that so the controller always has headroom.

const float TURN_AMOUNT_MMS = 200.0;            // how much the inside wheel slows when turning, mm/s. If speed - this falls below
                                                // V_MIN_MMS the inside wheel just stops (pivot); above that it keeps arcing
const unsigned long COMMAND_TIMEOUT_MS = 500;   // safety: stop if no command arrives for this long

unsigned long lastCommandTime = 0;
char lastCommand = 'X';

void setup() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(trigR, OUTPUT);
    pinMode(echoR, INPUT);
    pinMode(trigF, OUTPUT);
    pinMode(echoF, INPUT);
    pinMode(in5, INPUT);
    pinMode(in6, INPUT);
    pinMode(in7, INPUT);
    setupEncoders();
    Serial.begin(115200);
    Serial.setTimeout(20);   // don't wait long for a full line
    stopRobot();
    logInfo(Serial, "Ready for commands");
}

void loop() {
    updateEncoders();
    // Read one full command line if available
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            char cmd = toupper(line.charAt(0));
            int speed = constrain(line.substring(1).toInt(), 0, (int)V_MAX_MMS);   // mm/s
            handleCommand(cmd, speed);
            lastCommandTime = millis();
        }
    }

    // Failsafe: if the controller disconnects or stops sending, stop the robot
    if (lastCommand != 'X' && millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
        stopRobot();
        lastCommand = 'X';
        logError(Serial, "Command timeout - stopped");
    }

    if (lastCommand == 'F') {
        straightFollow(followSpeed);
    }

    if (lastCommand == 'M') {
        followMe(followSpeed);
    }

    if (lastCommand == 'L') {
        lineFollow(followSpeed);
    }

    if (lastCommand == 'W') driveStraight(followSpeed, +1);
    if (lastCommand == 'S') driveStraight(followSpeed, -1);

    static unsigned long lastRawPrint = 0;
    if (millis() - lastRawPrint > 200) {
        lastRawPrint = millis();
        printRawEncoders();
        printSpeedTelemetry();
    }
}

void handleCommand(char cmd, int speed) {
    if (speed > 0 && speed < V_MIN_MMS) speed = (int)V_MIN_MMS;   // the robot can't hold anything slower
    // Only log when the command changes (otherwise the heartbeat spams Serial)
    Serial.print("cmd=");  Serial.print(cmd);  Serial.print(" speed=");  Serial.println(speed);
    if (cmd != lastCommand) {
        logInfo(Serial, "Command received");
        if (cmd == 'W' || cmd == 'S') resetStraight();
        if (cmd == 'F') resetWallFollow();
        if (cmd == 'L') resetLineFollow();
        lastCommand = cmd;
    }

    switch (cmd) {
        case 'W': followSpeed = speed;                     break;
        case 'S': followSpeed = speed;                     break;
        case 'A': spinLeft(speed);                         break;
        case 'D': spinRight(speed);                        break;
        case 'Q': turnLeft(speed, TURN_AMOUNT_MMS);            break;
        case 'E': turnRight(speed, TURN_AMOUNT_MMS);           break;
        case 'Z': turnLeftBackward(speed, TURN_AMOUNT_MMS);    break;
        case 'C': turnRightBackward(speed, TURN_AMOUNT_MMS);   break;
        case 'X': stopRobot();                             break;
        case 'F': followSpeed = speed;                     break;
        case 'M': followSpeed = speed;                     break;  
        case 'L': followSpeed = speed;                     break;
        default:  stopRobot();                             break;
    }
}
