/**
 * @file RunMaze.ino
 * @brief "Maze mode": runs the three maze phases one after another, switching between them by itself.
 *
 *   Phase 1  MZ_LINE         Follow the white tape (lineFollow, Motor.ino).
 *       -> ends when the tape runs out (all 3 IR sensors off) AND there is a wall beside the robot
 *          (right ultrasonic sensor), seen several times in a row.
 *
 *   Phase 2  MZ_WALL         Follow the right-hand wall and its corners (straightFollow, Motor.ino).
 *       -> ends when the horizontal white tape that bounds the arena is under the robot (2+ IR sensors on it)
 *          and the wall has ended (wall lost / outside-corner logic running), or the tape has been seen
 *          for MZ_TAPE_GRACE_MM of travel anyway.
 *
 *   Phase 3  MZ_ARENA_ENTER  Roll MZ_ARENA_ENTER_MM straight ahead so the whole robot is past the tape.
 *            MZ_OBJECT       Object detection (ObjectDetect.ino): sweep, pick the biggest/closest object, approach it.
 *
 *   MZ_FINISHED              Stopped. Maze complete (or the object search gave up).
 *
 * Start it with the "O" command (the 'o' key in MyRobotControl.pde). Call runMaze() every pass of loop().
 *
 * Assumptions: the wall is on the robot's RIGHT (the only side sensor), the servo is centred in phases 1 and 2.
 *
 * This tab is named so it sorts after ObjectDetect.ino (Arduino joins tabs alphabetically, see ObjectDetect.ino).
 */

// ---------------------------------------------------------------------
// Tuning (starting guesses: TUNE)
// ---------------------------------------------------------------------
const int MZ_LINE = 0, MZ_WALL = 1, MZ_ARENA_ENTER = 2, MZ_OBJECT = 3, MZ_FINISHED = 4;
const int MZ_START_PHASE = MZ_LINE;          // set to MZ_WALL or MZ_OBJECT to test a later phase on its own

// Phase 1 -> 2
const float MZ_WALL_DETECT_CM  = 30.0;       // right wall closer than this = "there are walls beside me"
const int   MZ_WALL_CONFIRM    = 3;          // ... seen this many times in a row (30 ms apart) with no tape under the robot

// Phase 2 -> 3
const int   MZ_TAPE_MIN_SENSORS = 2;         // the horizontal tape must be under at least this many of the 3 IR sensors
const int   MZ_TAPE_CONFIRM     = 2;         // ... on this many checks in a row (10 ms apart)
const float MZ_P2_MIN_MM        = 150.0;     // ignore tape until the robot has driven this far in phase 2 (leftover tape from phase 1)
const float MZ_TAPE_GRACE_MM    = 60.0;      // if the wall hasn't visibly ended, still switch after this much travel over tape
const float MZ_ARENA_ENTER_MM   = 120.0;     // drive this far past the tape before searching for objects

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------
int   mzPhase       = MZ_FINISHED;
int   mzWallCount   = 0;                     // phase 1: consecutive "no tape + wall beside me" checks
int   mzTapeCount   = 0;                     // phase 2: consecutive "horizontal tape" checks
float mzPhaseStartMm = 0;                    // odoDist when the phase began
float mzTapeAtMm    = 0;                     // odoDist when the tape was first confirmed (-1e9 = not yet)
unsigned long mzLastCheck = 0;

/** How many of the 3 IR sensors currently see tape. */
int mzTapeSensors() {
    return (lineSensorOnTape(in5) ? 1 : 0) + (lineSensorOnTape(in6) ? 1 : 0) + (lineSensorOnTape(in7) ? 1 : 0);
}

void mzEnterPhase(int p) {
    mzPhase = p;
    mzWallCount = 0;
    mzTapeCount = 0;
    mzTapeAtMm = -1e9f;
    mzPhaseStartMm = odoDist;
    mzLastCheck = 0;
    Serial.print("maze phase="); Serial.println(p);

    switch (p) {
        case MZ_LINE:
            scanServoCenter();
            resetLineFollow();
            break;
        case MZ_WALL:
            scanServoCenter();
            resetWallFollow();               // captures a fresh wall distance as the target
            break;
        case MZ_ARENA_ENTER:
            scanServoCenter();
            markMotion();
            resetStraight();
            break;
        case MZ_OBJECT:
            stopRobot();
            resetObjectDetect();
            break;
        case MZ_FINISHED:
        default:
            stopRobot();
            scanServoCenter();
            break;
    }
}

/** Call when maze mode starts (command change to 'O'). */
void resetMaze() {
    stopRobot();
    mzEnterPhase(MZ_START_PHASE);
}

/** @brief Run the maze. Call every pass of loop() while in maze mode. */
void runMaze(float speed) {
    unsigned long now = millis();
    float base = constrain(speed, V_MIN_MMS, V_MAX_MMS);

    switch (mzPhase) {

    // ---------------- Phase 1: line following ----------------
    case MZ_LINE: {
        if (now - mzLastCheck < 30) return;      // same rate as lineFollow()
        mzLastCheck = now;

        if (mzTapeSensors() == 0) {
            // No tape under the robot: is this the end of the line (walls beside us) or just a gap / sharp corner?
            // (The ultrasonic ping is only taken when the tape is gone, so it doesn't slow normal line following.)
            float d = readDistanceCm(trigR, echoR);
            if (d > 0 && d < MZ_WALL_DETECT_CM) {
                if (++mzWallCount >= MZ_WALL_CONFIRM) {
                    Serial.println("maze: line ended at the walls -> wall following");
                    mzEnterPhase(MZ_WALL);
                }
                return;                          // keep rolling on the last command while we confirm (don't start the search spin)
            }
        }
        mzWallCount = 0;
        lineFollow(base);
        break;
    }

    // ---------------- Phase 2: wall / corner following ----------------
    case MZ_WALL: {
        straightFollow(base);                    // rate-limits itself (60 ms)

        if (now - mzLastCheck < 10) return;
        mzLastCheck = now;

        bool tape = (mzTapeSensors() >= MZ_TAPE_MIN_SENSORS) && (odoDist - mzPhaseStartMm >= MZ_P2_MIN_MM);
        mzTapeCount = tape ? mzTapeCount + 1 : 0;
        if (mzTapeCount < MZ_TAPE_CONFIRM) return;

        if (mzTapeAtMm < -1e8f) mzTapeAtMm = odoDist;
        bool wallEnded = wfLost || wfState == WF_OUTSIDE_FWD || wfState == WF_OUTSIDE_TURN || wfState == WF_OUTSIDE_FIND;
        if (wallEnded || odoDist - mzTapeAtMm >= MZ_TAPE_GRACE_MM) {
            Serial.println("maze: arena line reached -> object detection");
            mzEnterPhase(MZ_ARENA_ENTER);
        }
        break;
    }

    // ---------------- Phase 3a: roll into the arena ----------------
    case MZ_ARENA_ENTER:
        if (travelledMm() >= MZ_ARENA_ENTER_MM) mzEnterPhase(MZ_OBJECT);
        else                                    driveStraight(base, +1);
        break;

    // ---------------- Phase 3b: object detection ----------------
    case MZ_OBJECT: {
        int r = objectDetect(base);
        if (r == OD_REACHED) { Serial.println("MAZE COMPLETE"); mzEnterPhase(MZ_FINISHED); }
        if (r == OD_FAILED)  { Serial.println("maze: object search failed"); mzEnterPhase(MZ_FINISHED); }
        break;
    }

    case MZ_FINISHED:
    default:
        break;
    }
}
