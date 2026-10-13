// MyRobot-V1.ino
// This file must be named the same as your sketch folder
//
// Remote control over Serial. The Processing sketch (MyRobotControl.pde)
// sends lines like "W 200\n" = <command letter> <space> <speed 0-255>.
//
//   W = forward            S = backward
//   A = spin left          D = spin right        (in place)
//   Q = forward + left     E = forward + right   (turn while driving)
//   Z = backward + left    C = backward + right
//   X = stop

int enA = 9;   // Enable pin for Motor A (LEFT) - must be a PWM-capable pin         !!! NOTE!! CHANGE PINS ACCORDING TO OUR WIRING!!!
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A
int enB = 6;   // Enable pin for Motor B (RIGHT) - must be a PWM-capable pin
int in3 = 5;   // Direction control pin 3 for Motor B
int in4 = 4;   // Direction control pin 4 for Motor B

const int TURN_AMOUNT = 120;                    // how much the inside wheel slows when turning
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
    Serial.begin(9600);
    Serial.setTimeout(20);   // don't wait long for a full line
    stopRobot();
    logInfo(Serial, "Ready for commands");
}

void loop() {
    // Read one full command line if available
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            char cmd = toupper(line.charAt(0));
            int speed = constrain(line.substring(1).toInt(), 0, 255);
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
}

void handleCommand(char cmd, int speed) {
    // Only log when the command changes (otherwise the heartbeat spams Serial)
    if (cmd != lastCommand) {
        logInfo(Serial, "Command received");
        lastCommand = cmd;
    }

    switch (cmd) {
        case 'W': driveForward(speed);                     break;
        case 'S': driveBackward(speed);                    break;
        case 'A': spinLeft(speed);                         break;
        case 'D': spinRight(speed);                        break;
        case 'Q': turnLeft(speed, TURN_AMOUNT);            break;
        case 'E': turnRight(speed, TURN_AMOUNT);           break;
        case 'Z': turnLeftBackward(speed, TURN_AMOUNT);    break;
        case 'C': turnRightBackward(speed, TURN_AMOUNT);   break;
        case 'X':
        default:  stopRobot();                             break;
    }
}
