// MyRobotControl.pde  (Processing 4)
// WASD keyboard control for the Arduino robot over USB serial.
//
//   W / S      forward / backward
//   A / D      spin left / right (in place)
//   W+A, W+D   turn while driving forward
//   S+A, S+D   turn while driving backward
//   UP / DOWN  speed up / slow down
//   SPACE      emergency stop
//   F          begin wall following
//   M          follow me
//   G          object detection (sweep, then approach the best object)
//   O          MAZE MODE: line follow -> wall follow -> object detection, switches phases by itself
//   R          re-scan for the robot (if it wasn't found or got unplugged)
//
// IMPORTANT: close the Arduino Serial Monitor first, only one program
// can use the port at a time.

import processing.serial.*;

// ---- Settings ----
String PORT_NAME = "";        // optional manual override, e.g. "COM5" or "/dev/cu.usbmodem1101". Leave "" to auto-detect.
int BAUD = 115200;             // must match Serial.begin() on the Arduino
int PROBE_MS = 2500;         // how long to listen on each port for the robot's output while auto-detecting
int HEARTBEAT_MS = 100;
int SPEED_STEP = 10;         // mm/s per key press
int MIN_SPEED = 60;          // mm/s. Must match V_MIN_MMS in MyRobot-V1.ino (the slowest the robot can hold)
int MAX_SPEED = 400;         // mm/s. Must match V_MAX_MMS in MyRobot-V1.ino

Serial port;
boolean w, a, s, d;
boolean following = false;
boolean followingMe = false;
boolean lineFollowing = false;
boolean objectDetecting = false;
boolean mazeMode = false;
int speed = 80;              // mm/s
String lastSent = "";
int lastSendTime = 0;
String status = "";
int lastRxTime = 0;       // last time the robot sent us anything

// ---- Coolness (visuals) ----
color IDLE_COL = color(70, 120, 220);    // waves when stopped
color FWD_COL  = color(60, 210, 150);    // driving forward
color BACK_COL = color(255, 150, 70);    // driving backward
color SPIN_COL = color(170, 110, 240);   // spinning in place
color waveCol;
float wavePhase = 0;      // advances every frame; faster when the robot is faster
float activity = 0;       // smoothed 0..1: how fast the robot is going right now
float flow = 1;           // smoothed -1..1: wave direction (reverses when backing up)
char[] KONAMI = {'w', 'w', 's', 's', 'a', 'd', 'a', 'd'};
int konamiIndex = 0;     // how far through the sequence you are
int konamiUntil = 0;     // millis() time when the effect ends

ArrayList<Confetto> confetti = new ArrayList<Confetto>();
color[] CONFETTI_COLS = {
  color(255, 90, 90), color(255, 200, 60), color(90, 220, 140),
  color(80, 170, 255), color(190, 120, 255), color(255, 130, 200)
};

class Confetto {
  float x, y, vx, vy, rot, vrot, flip, vflip, size;
  color c;

  Confetto(float x, float y, float vx, float vy) {
    this.x = x;  this.y = y;
    this.vx = vx;  this.vy = vy;
    rot  = random(TWO_PI);  vrot  = random(-0.2, 0.2);
    flip = random(TWO_PI);  vflip = random(0.08, 0.25);
    size = random(4, 8) * (height / 340.0);      // scales with window size
    c = CONFETTI_COLS[int(random(CONFETTI_COLS.length))];
  }

  void update() {
    vx *= 0.99;                       // air resistance
    vy = min(vy + 0.1, 3.0);          // gravity, with a terminal velocity
    x += vx + sin(flip) * 0.5;        // slight side-to-side flutter
    y += vy;
    rot += vrot;
    flip += vflip;
  }

  boolean offscreen() {
    return y > height + 20;
  }

  void show() {
    pushMatrix();
    translate(x, y);
    rotate(rot);
    fill(c);
    // height shrinks and grows with abs(sin(flip)) so it looks like it's tumbling
    rect(0, 0, size, size * 0.6 * abs(sin(flip)) + 1);
    popMatrix();
  }
}

void setup() {
  size(840, 680);
  textAlign(CENTER, CENTER);
  textFont(createFont("SansSerif", 28, true));
  waveCol = IDLE_COL;

  connectRobot();
}

// Is this port name one that usually belongs to an Arduino?
boolean looksLikeArduino(String name) {
  return name.contains("usbmodem") || name.contains("ttyACM") ||
         name.contains("ttyUSB")   || name.contains("usbserial") || name.startsWith("COM");
}

// Open `name` and check that the robot is talking on it. The robot prints "rawA=... rawB=..."
// about 5 times a second, so listening for that identifies it. Returns the open port, or null.
Serial probePort(String name) {
  Serial p = null;
  try {
    p = new Serial(this, name, BAUD);
    p.clear();
    int t0 = millis();
    while (millis() - t0 < PROBE_MS) {
      while (p.available() > 0) {
        String line = p.readStringUntil('\n');
        if (line == null) break;                       // partial line, wait for the rest
        if (line.contains("rawA=") || line.contains("[INFO]") || line.contains("cmd=")) {
          return p;                                    // that's our robot
        }
      }
      delay(20);
    }
  } catch (Exception e) {
    println("  could not open " + name + ": " + e.getMessage() + "  (is the Serial Monitor open?)");
  }
  if (p != null) p.stop();                             // not the robot: release the port
  return null;
}

// Find the robot: try the manual override first, then Arduino-looking ports, then the rest.
void connectRobot() {
  if (port != null) { port.stop(); port = null; }

  String[] ports = Serial.list();
  println("Available serial ports:");
  printArray(ports);
  if (ports.length == 0) {
    status = "No serial ports found - is the robot plugged in? (press R to retry)";
    return;
  }

  ArrayList<String> order = new ArrayList<String>();
  if (PORT_NAME.length() > 0) order.add(PORT_NAME);
  for (String p : ports) if (looksLikeArduino(p) && !order.contains(p)) order.add(p);
  for (String p : ports) if (!order.contains(p)) order.add(p);

  for (String name : order) {
    if (name.contains("/tty.")) continue;                          // macOS duplicate of the /cu. port
    if (name.toLowerCase().contains("bluetooth")) continue;        // can hang for a long time
    println("Looking for the robot on " + name + " ...");
    Serial p = probePort(name);
    if (p != null) {
      port = p;
      status = "Connected: " + name;
      lastRxTime = millis();
      println("Robot found on " + name);
      return;
    }
  }
  status = "Robot not found - check USB / close Serial Monitor, then press R";
  println(status);
}

void draw() {
  drawWaves();

  // Print anything the Arduino logs back (from logInfo / logError)
  if (port != null) {
    while (port.available() > 0) {
      String msg = port.readStringUntil('\n');
      if (msg == null) break;
      lastRxTime = millis();
      print("Arduino: " + msg);
    }
  }

  // Send on change, plus a regular heartbeat
  String cmd = computeCommand() + " " + speed;
  if (!cmd.equals(lastSent) || millis() - lastSendTime > HEARTBEAT_MS) {
    send(cmd);
  }
  drawConfetti();
  drawUI();
}

// Turn the held keys into one command letter (matches MyRobot-V1.ino)
String computeCommand() {
  if (mazeMode) return "O";
  if (objectDetecting) return "G";
  if (lineFollowing) return "L";
  if (following) return "F";
  if (followingMe) return "M";
  
  boolean fwd   = w && !s;
  boolean back  = s && !w;
  boolean left  = a && !d;
  boolean right = d && !a;

  if (fwd && left)   return "Q";
  if (fwd && right)  return "E";
  if (back && left)  return "Z";
  if (back && right) return "C";
  if (fwd)           return "W";
  if (back)          return "S";
  if (left)          return "A";
  if (right)         return "D";
  return "X";
}

void send(String cmd) {
  if (port != null) {
    port.write(cmd + "\n");
  }
  lastSent = cmd;
  lastSendTime = millis();
}

// Switch every autonomous mode off (the start functions below then switch their own one on)
void clearAutoModes() {
  following = false;
  followingMe = false;
  lineFollowing = false;
  objectDetecting = false;
  mazeMode = false;
}

void toggleObjectDetect() {
  w = a = s = d = false;
  boolean on = !objectDetecting;
  clearAutoModes();
  objectDetecting = on;
  println(on ? "Object detection ON" : "Object detection OFF");
}

void toggleMazeMode() {
  w = a = s = d = false;
  boolean on = !mazeMode;
  clearAutoModes();
  mazeMode = on;
  println(on ? "MAZE MODE ON (line -> wall -> object)" : "MAZE MODE OFF");
}

void startStraightFollow() {
  if (following) return;
  w = a = s = d = false;
  clearAutoModes();
  following = true;
  println("Wall follow ON (right wall)");
}

void stopStraightFollow() {
  if (!following) return;
  following = false;
  println("Wall follow OFF");
}

void startFollowMe() {
  if (followingMe) return;
  w = a = s = d = false;
  clearAutoModes();
  followingMe = true;
  println("Follow me ON");
}

void toggleLineFollow() {
  w = a = s = d = false;
  boolean on = !lineFollowing;
  clearAutoModes();
  lineFollowing = on;
  println(on ? "IR line follow ON" : "IR line follow OFF");
}

void stopFollowMe() {
  if (!followingMe) return;
  followingMe = false;
  println("Follow me OFF");
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP)   speed = min(MAX_SPEED, speed + SPEED_STEP);
    if (keyCode == DOWN) speed = max(MIN_SPEED, speed - SPEED_STEP);
    return;
  }
  char k = Character.toLowerCase(key);

  boolean isDriveKey = (k == 'w' || k == 'a' || k == 's' || k == 'd');


  // Konami check: only on a fresh press, not on auto-repeat while a key is held
  boolean alreadyHeld = (k == 'w' && w) || (k == 'a' && a) || (k == 's' && s) || (k == 'd' && d);
  if (!alreadyHeld && isDriveKey) checkKonami(k);

  if (k == 'r') { w = a = s = d = false; connectRobot(); return; }   // re-scan for the robot
  if (k == 'f') { startStraightFollow(); return; }
  if (k == 'm') { startFollowMe(); return; }
  if (k == 'l') { toggleLineFollow(); return; }
  if (k == 'g') { toggleObjectDetect(); return; }
  if (k == 'o') { toggleMazeMode(); return; }
  if (isDriveKey) { stopStraightFollow(); stopFollowMe(); clearAutoModes(); }

  if (k == 'w') w = true;
  if (k == 'a') a = true;
  if (k == 's') s = true;
  if (k == 'd') d = true;
  if (k == ' ') { w = a = s = d = false; clearAutoModes(); }   // emergency stop
}

void keyReleased() {
  if (key == CODED) return;
  char k = Character.toLowerCase(key);
  if (k == 'w') w = false;
  if (k == 'a') a = false;
  if (k == 's') s = false;
  if (k == 'd') d = false;
}

void checkKonami(char k) {
  if (k == KONAMI[konamiIndex]) {
    konamiIndex++;
    if (konamiIndex == KONAMI.length) {      // finished the sequence
      konamiIndex = 0;
      konamiUntil = millis() + 6000;         // party for 6 seconds
      spawnBurst();
      println("Party time!");
    }
  } else {
    // wrong key: start over, but a 'w' can count as the first key of a new try
    konamiIndex = (k == KONAMI[0]) ? 1 : 0;
  }
}

// A smoothly cycling rainbow color. offset shifts the hue (used per wave layer).
color rainbow(float offset) {
  float t = millis() * 0.003 + offset;
  return color(128 + 127 * sin(t),
               128 + 127 * sin(t + 2.094),
               128 + 127 * sin(t + 4.188));
}

// Fan of confetti shooting up from the middle of the window
void spawnBurst() {
  for (int i = 0; i < 140; i++) {
    float angle = random(-PI * 0.9, -PI * 0.1);   // upward fan
    float spd = random(2, 7);
    confetti.add(new Confetto(width / 2, height * 0.55, cos(angle) * spd, sin(angle) * spd));
  }
}

void drawConfetti() {
  // Steady rain from the top while Konami mode is active
  if (millis() < konamiUntil) {
    for (int i = 0; i < 3; i++) {
      confetti.add(new Confetto(random(width), -10, random(-1, 1), random(0.5, 2)));
    }
  }

  pushStyle();
  noStroke();
  rectMode(CENTER);
  for (int i = confetti.size() - 1; i >= 0; i--) {   // backwards so removing is safe
    Confetto p = confetti.get(i);
    p.update();
    p.show();
    if (p.offscreen()) confetti.remove(i);
  }
  popStyle();   // restores rectMode(CORNER) etc. for the rest of the drawing
}

void exit() {
  send("X 0");   // make sure the robot stops when the window closes
  super.exit();
}

// ---- Drawing ----
// Animated background: a dark gradient with layered translucent waves.
// Speed and height of the waves follow how fast the robot is moving,
// the color follows what it's doing, and the flow reverses when backing up.
void drawWaves() {
  boolean party = millis() < konamiUntil;
  String c = computeCommand();
  boolean moving = !c.equals("X");

  // Targets for this frame
  float targetActivity = moving ? map(speed, MIN_SPEED, MAX_SPEED, 0.25, 1.0) : 0;
  color targetCol = IDLE_COL;
  float targetFlow = 1;
  if (c.equals("W") || c.equals("Q") || c.equals("E")) targetCol = FWD_COL;
  if (c.equals("S") || c.equals("Z") || c.equals("C")) { targetCol = BACK_COL; targetFlow = -1; }
  if (c.equals("A") || c.equals("D")) targetCol = SPIN_COL;
  if (c.equals("W") || c.equals("Q") || c.equals("E") || c.equals("F") || c.equals("M") || c.equals("L") || c.equals("G") || c.equals("O")) targetCol = FWD_COL;

  // Ease toward the targets so changes feel smooth, not abrupt
  activity = lerp(activity, targetActivity, 0.06);
  flow     = lerp(flow, targetFlow, 0.05);
  waveCol  = lerpColor(waveCol, targetCol, 0.06);
  if (party) waveCol = rainbow(0);   // keys and speed bar change color too
  if (party) activity = lerp(activity, 1.0, 0.1);   // full-energy waves
  wavePhase += (0.015 + activity * 0.09) * flow;

  // Background gradient
  noFill();
  strokeWeight(1);
  color top = color(14, 18, 34);
  color bottom = color(26, 34, 62);
  for (int y = 0; y < height; y++) {
    stroke(lerpColor(top, bottom, y / (float) height));
    line(0, y, width, y);
  }

  // Wave layers (back to front)
  noStroke();
  float amp = 8 + activity * 28;
  for (int layer = 0; layer < 4; layer++) {
    float baseY = height * (0.55 + layer * 0.14);
    float freq  = 0.010 + layer * 0.004;
    float phase = wavePhase * (1 + layer * 0.35) + layer * 1.7;
    color layerCol = party ? rainbow(layer * 0.8) : waveCol;   // each layer a different hue
    fill(layerCol, 28 + layer * 14 + (party ? 30 : 0));
    beginShape();
    vertex(0, height);
    for (int x = 0; x <= width; x += 6) {
      float y = baseY
              + sin(x * freq + phase) * amp
              + sin(x * freq * 2.3 - phase * 0.7) * amp * 0.4;
      vertex(x, y);
    }
    vertex(width, height);
    endShape(CLOSE);
  }
}

void drawKey(String label, boolean pressed, float x, float y) {
  noStroke();
  if (pressed) {                                   // soft glow
    for (int i = 4; i > 0; i--) {
      fill(waveCol, 20);
      rect(x - i * 4, y - i * 4, 70 + i * 8, 70 + i * 8, 14 + i * 3);
    }
  }
  strokeWeight(1.5);
  stroke(255, pressed ? 220 : 70);
  if (pressed) fill(waveCol, 200); else fill(255, 18);
  rect(x, y, 70, 70, 14);
  noStroke();
  fill(255, pressed ? 255 : 190);
  textSize(28);
  text(label, x + 35, y + 33);
}

void drawUI() {
  rectMode(CORNER);

  // Fit the original 420x340 layout into whatever size the window is
  float k = min(width / 420.0, height / 340.0);
  pushMatrix();
  translate((width - 420 * k) / 2, (height - 340 * k) / 2);
  scale(k);

  // Title
  fill(255, 170);
  textSize(13);
  text(millis() < konamiUntil ? "P A R T Y   T I M E !" 
       : following ? "W A L L   F O L L O W" 
       : followingMe ? " F O L L O W    M E"
       : mazeMode ? "M A Z E   M O D E"
       : objectDetecting ? "O B J E C T   D E T E C T I O N"
       : lineFollowing ? " I R   L I N E   F O L L O W"
       : "B R U B R U", 210, 24);

  // Keys
  drawKey("W", w, 175, 50);
  drawKey("A", a, 95, 130);
  drawKey("S", s, 175, 130);
  drawKey("D", d, 255, 130);

  // Speed bar
  float bx = 60, bw = 300, by = 232, bh = 8;
  noStroke();
  fill(255, 30);
  rect(bx, by, bw, bh, 4);
  fill(waveCol, 230);
  rect(bx, by, bw * speed / (float) MAX_SPEED, bh, 4);
  fill(255, 220);
  textSize(15);
  text("SPEED  " + speed + " mm/s", 210, by + 26);

  // Footer
  fill(255, 120);
  textSize(11);
  text("SPACE stop  F wall  M follow  L line  G object  O maze  R reconnect", 210, 288);
  text("sending: " + lastSent, 210, 305);
  boolean silent = (port != null && millis() - lastRxTime > 1500);
  text(silent ? status + "   (NO DATA from robot!)" : status, 210, 322);

  popMatrix();
} 
  
