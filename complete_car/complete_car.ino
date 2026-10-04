/*
 * ============================================================================
 *  4WD AUTONOMOUS CAR  —  SINGLE COMPLETE FIRMWARE  (USE THIS ONE FILE)
 * ============================================================================
 *  This ONE program contains everything, in the order you will test it:
 *
 *    [1] 230° SERVO on D10  +  HC-SR04 ULTRASONIC on D8/D9  (radar scan)
 *    [2] 2x IR SENSORS on A0 / A1                            (reflex bumpers)
 *    [3] L298N 4WD MOTORS                                    (D2..D7, ENA=D5, ENB=D6)
 *    [4] ESP32-CAM + LAPTOP ML MODEL as an ADD-ON            (serial in on A2)
 *
 * ----------------------------------------------------------------------------
 *  FOUR MODES  (change mode by typing one letter in the Serial Monitor,
 *               or by calling the ESP32-CAM URL /cmd?c=<LETTER>)
 * ----------------------------------------------------------------------------
 *    'M' = MANUAL  : you drive with the keyboard (W A S D S) — used to TEST
 *    'A' = AUTO    : cars drives itself using servo + ultrasonic + 2x IR
 *    'C' = CAMERA  : the LAPTOP ML MODEL drives; ultrasonic + IR stay active
 *                    as a hard safety override so it can never crash
 *    'S' = STOP    : motors off immediately
 *
 *    Movement letters:  W=forward  Q=curve left  E=curve right
 *                       A=spin left  D=spin right  B=reverse  S=stop
 *    Speed digits: '1'(slow) .. '9'(fast)   'H'=help   '?'=status
 *
 * ----------------------------------------------------------------------------
 *  SERIAL MONITOR: 9600 baud      ESP32-CAM LINK: 9600 baud on A2
 *  The ESP32-CAM UART is on A2/A3, NOT on D0/D1, so you never have to
 *  unplug any wire to upload new code.
 * ============================================================================
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// ---------------------------------------------------------------- PIN MAP ---
#define PIN_IN1        2      // L298N IN1  -> Left motors forward
#define PIN_IN2        3      // L298N IN2  -> Left motors backward
#define PIN_IN3        4      // L298N IN3  -> Right motors forward
#define PIN_IN4        7      // L298N IN4  -> Right motors backward
#define PIN_ENA        5      // L298N ENA  -> Left speed  (Timer0 PWM, safe with Servo)
#define PIN_ENB        6      // L298N ENB  -> Right speed (Timer0 PWM, safe with Servo)
#define PIN_TRIG       8      // HC-SR04 TRIG
#define PIN_ECHO       9      // HC-SR04 ECHO
#define PIN_SERVO     10      // 230 degree servo signal
#define PIN_IR_LEFT   A0      // Left  IR sensor OUT
#define PIN_IR_RIGHT  A1      // Right IR sensor OUT
#define PIN_CAM_RX    A2      // ESP32-CAM U0T (GPIO1) sends INTO the Uno here
#define PIN_CAM_TX    A3      // optional telemetry back to ESP32-CAM U0R

// --------------------------------------------------------------- SETTINGS ---
// Servo is a 230 degree unit: 500us = 0 deg, 2500us = 230 deg.
// Physical straight-ahead centre of a 230 deg servo is 115 degrees = 1500 us.
#define SERVO_MIN_US      500
#define SERVO_MAX_US     2500
#define SERVO_MAX_ANGLE   230
#define SERVO_CENTER      115

#define SPEED_CRUISE      165   // normal driving PWM
#define SPEED_SLOW        125   // PWM when something is getting close
#define SPEED_TURN        155   // PWM while turning
#define SPEED_REVERSE     150

#define DIST_SLOW_CM       45   // start slowing down below this
#define DIST_STOP_CM       28   // stop + radar scan below this
#define DIST_SAFETY_CM     20   // emergency stop (hard override, all modes)

#define IR_ACTIVE_LOW       1   // 1 = most IR modules (LOW when obstacle seen)
#define REFLEX_DODGE_MS   240   // length of an instant IR dodge

#define ML_WATCHDOG_MS    900   // no ML command for this long -> stop the car

#define LOOP_MS            20   // main loop period (safety check every 20 ms)

#define SELF_TEST_MOTORS    0   // keep 0 while running on USB power only!
                                // set to 1 once the battery / motor supply exists

// Radar scan angles across the 230 degree arc (0 = fully right, 230 = fully left)
const int RADAR_ANGLES[] = { 30, 65, 115, 165, 200 };
const int RADAR_COUNT    = 5;

// ------------------------------------------------------------- GLOBAL STATE --
Servo radarServo;
SoftwareSerial camSerial(PIN_CAM_RX, PIN_CAM_TX);

enum CarMode { MODE_STOP, MODE_MANUAL, MODE_AUTO, MODE_CAMERA };
CarMode mode = MODE_STOP;

int  driveSpeed   = SPEED_CRUISE;
int  servoAngle   = SERVO_CENTER;

float distanceCm  = 999.0;
bool  irLeft      = false;
bool  irRight     = false;

unsigned long lastCamCmdMs  = 0;
unsigned long lastLoopMs    = 0;
char          lastCamCmd    = 'S';

// ---- forward declarations (so the code also compiles as a plain .cpp) ------
void  driveMotors(int leftSpeed, int rightSpeed);
void  stopMotors();
int   innerWheelSpeed();
void  forward();  void backward();
void  curveLeft(); void curveRight();
void  spinLeft(); void spinRight();
void  moveServo(int angleDeg);
float readDistanceCm();
float readDistanceCmMedian();
void  readSensors();
void  sensorAutoDrive();
void  emergencyAvoid();
void  cameraMlDrive();
void  handleIncoming();
void  processChar(char c, bool fromCam);
void  printHelp();

// ============================================================================
//                                   SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  camSerial.begin(9600);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);  pinMode(PIN_IR_RIGHT, INPUT);

  stopMotors();

  radarServo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  moveServo(SERVO_CENTER);

  printHelp();

  // --- boot self-test: proves servo + ultrasonic + IR are alive -------------
  Serial.println(F("--- BOOT SELF TEST ---"));
  Serial.print(F("Centering 230deg servo at ")); Serial.print(SERVO_CENTER);
  Serial.println(F(" deg (1500 us) ..."));
  delay(700);
  Serial.println(F("Sweeping Right(65) -> Center(115) -> Left(165) ..."));
  moveServo(65);  delay(600);
  moveServo(115); delay(600);
  moveServo(165); delay(600);
  moveServo(SERVO_CENTER);
  delay(400);

  readSensors();
  Serial.print(F("Ultrasonic distance : ")); Serial.print(distanceCm); Serial.println(F(" cm"));
  Serial.print(F("Left  IR sensor     : ")); Serial.println(irLeft  ? F("OBSTACLE") : F("clear"));
  Serial.print(F("Right IR sensor     : ")); Serial.println(irRight ? F("OBSTACLE") : F("clear"));

#if SELF_TEST_MOTORS
  Serial.println(F("Motor pulse test (2s) ..."));
  driveMotors(120, 120); delay(2000); stopMotors();
#endif

  Serial.println(F("Self test done. Type 'M' to test motors by keyboard,"));
  Serial.println(F("'A' for sensor auto drive, 'C' for laptop ML driving."));
  Serial.println(F("Car is STOPPED until you choose a mode."));
}

// ============================================================================
//                                   LOOP
// ============================================================================
void loop() {
  handleIncoming();                       // read keyboard + ESP32-CAM chars

  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_MS) return;
  lastLoopMs = now;

  readSensors();                          // ultrasonic + both IR sensors

  switch (mode) {
    case MODE_MANUAL:  break;             // keys already moved the motors
    case MODE_AUTO:    sensorAutoDrive(); break;
    case MODE_CAMERA:  cameraMlDrive();   break;
    default:           stopMotors();      break;
  }
}

// ============================================================================
//                        MOTORS  (L298N, 4WD)
// ============================================================================
// leftSpeed / rightSpeed : -255 .. +255  (negative = wheel spins backward)
void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  digitalWrite(PIN_IN1, leftSpeed  > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, leftSpeed  < 0 ? HIGH : LOW);
  analogWrite(PIN_ENA, abs(leftSpeed));

  digitalWrite(PIN_IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, rightSpeed < 0 ? HIGH : LOW);
  analogWrite(PIN_ENB, abs(rightSpeed));
}

void stopMotors() { driveMotors(0, 0); }

// inner wheel of a curve must never fall below the motor's stall PWM (~70),
// otherwise the small TT gear motors just buzz instead of turning
int innerWheelSpeed() {
  int s = (driveSpeed * 45) / 100;
  return (s < 70) ? 70 : s;
}

void forward()            { driveMotors(driveSpeed, driveSpeed); }
void backward()           { driveMotors(-SPEED_REVERSE, -SPEED_REVERSE); }
void curveLeft()          { driveMotors(innerWheelSpeed(), driveSpeed); }
void curveRight()         { driveMotors(driveSpeed, innerWheelSpeed()); }
void spinLeft()           { driveMotors(-SPEED_TURN, SPEED_TURN); }
void spinRight()          { driveMotors(SPEED_TURN, -SPEED_TURN); }

// ============================================================================
//                        SERVO + ULTRASONIC RADAR
// ============================================================================
void moveServo(int angleDeg) {
  angleDeg = constrain(angleDeg, 0, SERVO_MAX_ANGLE);
  servoAngle = angleDeg;
  long us = map(angleDeg, 0, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
  radarServo.writeMicroseconds(us);
}

float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 25 ms timeout -> roughly 4 m max range
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (dur == 0) return 999.0;                 // no echo = nothing there
  return dur * 0.0343 / 2.0;
}

// median of 3 readings kills random ultrasonic noise
float readDistanceCmMedian() {
  float a = readDistanceCm();
  delay(6);
  float b = readDistanceCm();
  delay(6);
  float c = readDistanceCm();
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================================
//                             SENSOR READING
// ============================================================================
void readSensors() {
  distanceCm = readDistanceCmMedian();
  irLeft  = (digitalRead(PIN_IR_LEFT)  == (IR_ACTIVE_LOW ? LOW : HIGH));
  irRight = (digitalRead(PIN_IR_RIGHT) == (IR_ACTIVE_LOW ? LOW : HIGH));
}

// ============================================================================
//                     MODE 'A' :  SENSOR AUTONOMOUS DRIVE
// ============================================================================
void sensorAutoDrive() {

  // ---- 1. IR reflex: instant dodge, zero delay -------------------------
  if (irLeft && !irRight) {
    Serial.println(F("[IR] Left corner blocked -> reflex dodge RIGHT"));
    spinRight(); delay(REFLEX_DODGE_MS); return;
  }
  if (irRight && !irLeft) {
    Serial.println(F("[IR] Right corner blocked -> reflex dodge LEFT"));
    spinLeft(); delay(REFLEX_DODGE_MS); return;
  }
  if (irLeft && irRight) {
    Serial.println(F("[IR] Both corners blocked -> STOP + radar scan"));
    emergencyAvoid();
    return;
  }

  // ---- 2. Ultrasonic decision ------------------------------------------
  if (distanceCm >= DIST_SLOW_CM) {
    forward();                                  // road is clear
  } else if (distanceCm >= DIST_STOP_CM) {
    driveMotors(SPEED_SLOW, SPEED_SLOW);        // 28..45 cm : ease off
  } else {
    Serial.println(F("[ULTRA] Object closer than 28 cm -> STOP + radar scan"));
    emergencyAvoid();                           // under 28 cm : avoid
  }
}

// Stop, sweep the 5 radar angles, turn toward the widest gap, resume
void emergencyAvoid() {
  stopMotors();
  delay(120);
  backward();
  delay(320);
  stopMotors();
  delay(120);

  int bestAngle = SERVO_CENTER;
  float bestDist = -1.0;

  for (int i = 0; i < RADAR_COUNT; i++) {
    moveServo(RADAR_ANGLES[i]);
    delay(280);                                  // let the servo settle
    float d = readDistanceCmMedian();
    Serial.print(F("  radar ")); Serial.print(RADAR_ANGLES[i]);
    Serial.print(F(" deg -> ")); Serial.print(d); Serial.println(F(" cm"));

    // skip a side whose IR sensor is already blocked
    if (RADAR_ANGLES[i] < 100 && irRight) continue;
    if (RADAR_ANGLES[i] > 130 && irLeft)  continue;

    if (d > bestDist) { bestDist = d; bestAngle = RADAR_ANGLES[i]; }
  }

  moveServo(SERVO_CENTER);
  delay(150);

  if (bestDist >= DIST_STOP_CM) {
    if (bestAngle < 100) {
      Serial.println(F("-> Turning RIGHT"));
      spinRight(); delay(520);
    } else if (bestAngle > 130) {
      Serial.println(F("-> Turning LEFT"));
      spinLeft();  delay(520);
    } else {
      Serial.println(F("-> Path clear ahead"));
      forward(); delay(250);
    }
  } else {
    Serial.println(F("-> Boxed in! Stopping."));
    stopMotors();
    delay(400);
  }
}

// ============================================================================
//              MODE 'C' :  LAPTOP ML MODEL DRIVES  (with safety net)
// ============================================================================
void cameraMlDrive() {

  // ---- HARD SAFETY OVERRIDE: sensors always win over the ML model -------
  if (distanceCm <= DIST_SAFETY_CM || irLeft || irRight) {
    Serial.print(F("[SAFETY] ML command '"));
    Serial.print(lastCamCmd);
    Serial.println(F("' overridden by hardware sensor!"));
    emergencyAvoid();
    lastCamCmdMs = 0;                    // force a fresh ML command afterwards
    return;
  }

  // ---- watchdog: if Wi-Fi / laptop lags, stop instead of crashing -------
  if (lastCamCmdMs == 0 || millis() - lastCamCmdMs > ML_WATCHDOG_MS) {
    stopMotors();
    return;
  }

  // ---- apply the ML / laptop decision ----------------------------------
  char c = lastCamCmd;
  switch (c) {
    case 'W': forward();    break;
    case 'Q': curveLeft();  break;
    case 'E': curveRight(); break;
    case 'A': spinLeft();   break;
    case 'D': spinRight();  break;
    case 'B': backward();   break;
    default:  stopMotors(); break;
  }
}

// ============================================================================
//                     INCOMING CHARACTERS (keys + ESP32)
// ============================================================================
void handleIncoming() {
  while (Serial.available())    processChar((char)Serial.read(),    false); // keyboard
  while (camSerial.available()) processChar((char)camSerial.read(), true);  // ESP32-CAM
}

// KEEPING IT SIMPLE:
//   CAPITAL letters in the Serial Monitor = MODE keys : M / A / C / H / S
//   small   letters in the Serial Monitor = MOVEMENT : w q e a d b s
//   Letters coming from the ESP32-CAM       = MOVEMENT : F G I L R B S + A / C modes
void processChar(char c, bool fromCam) {
  if (c == '\n' || c == '\r' || c == ' ') return;

  // ---- speed, works from both sources -----------------------------------
  if (c >= '1' && c <= '9') {
    driveSpeed = map(c - '0', 1, 9, 90, 255);
    Serial.print(F(">> Speed set to ")); Serial.println(driveSpeed);
    return;
  }

  // ---- MODE keys (capital letters, from either source) ------------------
  switch (c) {
    case 'M':
      mode = MODE_MANUAL; stopMotors();
      Serial.println(F(">> MODE = MANUAL  (drive with w q e a d b s)"));
      return;
    case 'A':
      mode = MODE_AUTO; stopMotors();
      Serial.println(F(">> MODE = AUTO    (servo + ultrasonic + 2x IR)"));
      return;
    case 'C':
      mode = MODE_CAMERA; stopMotors(); lastCamCmdMs = 0;
      Serial.println(F(">> MODE = CAMERA  (laptop ML model + hardware safety net)"));
      return;
    case 'H': case '?':
      printHelp();
      return;
    case 'S':
      stopMotors();
      if (mode == MODE_CAMERA) { lastCamCmd = 'S'; lastCamCmdMs = millis(); }
      Serial.print(F(">> STOP (")); Serial.print(fromCam ? F("CAM") : F("PC")); Serial.println(F(")"));
      return;
  }

  if (mode != MODE_MANUAL && mode != MODE_CAMERA) return;  // AUTO = sensors decide

  // ---- translate everything into one internal movement letter ------------
  char mapped = 0;
  if (fromCam) {
    switch (c) {                       // ESP32-CAM / Python protocol
      case 'F': mapped = 'W'; break;   // forward
      case 'G': mapped = 'Q'; break;   // curve left
      case 'I': mapped = 'E'; break;   // curve right
      case 'L': mapped = 'A'; break;   // spin left
      case 'R': mapped = 'D'; break;   // spin right
      case 'B': mapped = 'B'; break;   // reverse
    }
  } else {
    switch (c) {                       // Serial Monitor keyboard
      case 'w': mapped = 'W'; break;
      case 'q': mapped = 'Q'; break;
      case 'e': mapped = 'E'; break;
      case 'a': mapped = 'A'; break;
      case 'd': mapped = 'D'; break;
      case 'b': mapped = 'B'; break;
      case 's': mapped = 'S'; break;   // small letter = stop
    }
  }
  if (mapped == 0) return;

  if (mapped == 'S') {                 // small-letter stop
    stopMotors();
    if (mode == MODE_CAMERA) { lastCamCmd = 'S'; lastCamCmdMs = millis(); }
    Serial.println(F(">> STOP (PC)"));
    return;
  }

  if (mode == MODE_CAMERA) {
    lastCamCmd   = mapped;             // applied safely inside cameraMlDrive()
    lastCamCmdMs = millis();
    return;
  }

  // MANUAL mode: act at once
  switch (mapped) {
    case 'W': forward();    break;
    case 'Q': curveLeft();  break;
    case 'E': curveRight(); break;
    case 'A': spinLeft();   break;
    case 'D': spinRight();  break;
    case 'B': backward();   break;
  }
  Serial.print(F(">> MANUAL move '")); Serial.print(mapped); Serial.println(F("'"));
}

void printHelp() {
  Serial.println();
  Serial.println(F("================ 4WD AUTONOMOUS CAR ================"));
  Serial.println(F(" CAPITAL letters = MODES:"));
  Serial.println(F("   M = MANUAL  - you drive with the keyboard"));
  Serial.println(F("   A = AUTO    - servo + ultrasonic + 2x IR drive the car"));
  Serial.println(F("   C = CAMERA  - laptop ML model drives (safety net ON)"));
  Serial.println(F("   S = STOP now          H = this help"));
  Serial.println(F(" small letters = MOVES:"));
  Serial.println(F("   w forward | q curve left | e curve right"));
  Serial.println(F("   a spin left | d spin right | b reverse | s stop"));
  Serial.println(F(" SPEED: 1 (slow) ... 9 (fast)"));
  Serial.println(F("==================================================="));
}
