/*
 * ============================================================================
 * PHASE 1: 4WD AUTONOMOUS CAR — ULTRASONIC + 230° SERVO RADAR OBSTACLE AVOIDANCE
 * ============================================================================
 * Hardware in Phase 1:
 *   - Arduino Uno R3
 *   - L298N Dual H-Bridge Motor Driver + 4x TT DC Gear Motors (4WD Chassis)
 *   - HC-SR04 Ultrasonic Distance Sensor
 *   - 230° Rotation Servo Motor (mounted with HC-SR04 on top)
 *   - LM2596 Buck Converter (adjusted to 5.0V DC output)
 *   - 7.4V - 12V Battery Pack
 *
 * Pin Mapping (Matches schematics/full_wiring_schematic.png):
 *   - D5  -> L298N ENA (Left Motors Speed PWM - Timer0)
 *   - D6  -> L298N ENB (Right Motors Speed PWM - Timer0)
 *   - D7  -> 230° Servo SIGNAL
 *   - D8  -> L298N IN1 (Left Motors Forward)
 *   - D9  -> L298N IN2 (Left Motors Backward)
 *   - D10 -> L298N IN3 (Right Motors Forward)
 *   - D11 -> L298N IN4 (Right Motors Backward)
 *   - D12 -> HC-SR04 TRIG
 *   - D13 <- HC-SR04 ECHO
 *   (D0/D1 stay free for USB upload + Serial Monitor.)
 *
 * Why ENA/ENB are on D5 & D6:
 *   Arduino's <Servo.h> library uses Timer1, which disables PWM on D9 and D10.
 *   Keeping ENA/ENB on D5 and D6 (Timer0) guarantees smooth PWM motor speed!
 *   IN2 (D9) and IN3 (D10) only ever use digitalWrite(), which Servo.h does
 *   not affect, so they are safe where they are.
 * ============================================================================
 */

#include <Servo.h>

// -------------------- MOTOR DRIVER PINS (L298N) --------------------
const int PIN_IN1 = 8;    // Left motors forward
const int PIN_IN2 = 9;    // Left motors backward
const int PIN_IN3 = 10;   // Right motors forward
const int PIN_IN4 = 11;   // Right motors backward
const int PIN_ENA = 5;    // Left motors PWM speed  (Remove 5V jumper on L298N ENA!)
const int PIN_ENB = 6;    // Right motors PWM speed (Remove 5V jumper on L298N ENB!)

// -------------------- ULTRASONIC & SERVO PINS ----------------------
const int PIN_TRIG  = 12;
const int PIN_ECHO  = 13;
const int PIN_SERVO = 7;

// -------------------- 230-DEGREE SERVO CALIBRATION -----------------
// Physical range: 0° (extreme right-back) to 230° (extreme left-back)
// Physical dead-ahead center is 230 / 2 = 115° (1500 microseconds pulse)
const int SERVO_MIN_US       = 500;
const int SERVO_MAX_US       = 2500;
const int ANGLE_FAR_RIGHT    = 30;   // 85° right of center
const int ANGLE_RIGHT        = 65;   // 50° right of center
const int ANGLE_CENTER       = 115;  // Dead-ahead center (1500 us)
const int ANGLE_LEFT         = 165;  // 50° left of center
const int ANGLE_FAR_LEFT     = 200;  // 85° left of center

// -------------------- DRIVING & DISTANCE THRESHOLDS ----------------
const int STOP_DISTANCE_CM   = 28;   // Stop & scan if obstacle closer than 28 cm
const int SLOW_DISTANCE_CM   = 45;   // Slow down when approaching between 28-45 cm
const int MIN_CLEAR_PATH_CM  = 22;   // Minimum acceptable side clearance to turn into
const int MAX_SENSOR_DIST_CM = 300;  // Cap out-of-range ultrasonic readings at 300 cm

const int SPEED_CRUISE       = 165;  // Normal forward speed (0 - 255)
const int SPEED_APPROACH     = 125;  // Reduced speed when obstacle is 28-45 cm ahead
const int SPEED_REVERSE      = 150;  // Reverse speed
const int SPEED_TURN         = 190;  // 4WD skid-steer turning requires slightly higher torque

// Left/Right trim offset in case one side of motors spins slightly faster (-30 to +30)
const int LEFT_MOTOR_TRIM    = 0;
const int RIGHT_MOTOR_TRIM   = 0;

Servo radarServo;

// ============================================================================
// HELPER: Move 230-Degree Servo to Exact Physical Angle (0° to 230°)
// ============================================================================
void writeServo230(int physicalAngle) {
  physicalAngle = constrain(physicalAngle, 0, 230);
  long pulseUs = map(physicalAngle, 0, 230, SERVO_MIN_US, SERVO_MAX_US);
  radarServo.writeMicroseconds((int)pulseUs);
}

// ============================================================================
// HELPER: Read Distance from HC-SR04 Ultrasonic Sensor (in cm)
// ============================================================================
int readSinglePingCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 25,000 us timeout corresponds to ~425 cm max round-trip
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (duration == 0) {
    return MAX_SENSOR_DIST_CM; // No echo = wide open space ahead
  }
  int cm = (int)(duration * 0.0343 / 2.0);
  return constrain(cm, 2, MAX_SENSOR_DIST_CM);
}

int getFilteredDistanceCm() {
  int r1 = readSinglePingCm();
  delay(12);
  int r2 = readSinglePingCm();
  delay(12);
  int r3 = readSinglePingCm();

  // Return median of 3 readings to reject ultrasonic noise spikes
  if ((r1 <= r2 && r2 <= r3) || (r3 <= r2 && r2 <= r1)) return r2;
  if ((r2 <= r1 && r1 <= r3) || (r3 <= r1 && r1 <= r2)) return r1;
  return r3;
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS (4WD Skid-Steer via L298N)
// ============================================================================
void setMotorPWM(int leftSpeed, int rightSpeed) {
  int l = constrain(leftSpeed + LEFT_MOTOR_TRIM, 0, 255);
  int r = constrain(rightSpeed + RIGHT_MOTOR_TRIM, 0, 255);
  analogWrite(PIN_ENA, l);
  analogWrite(PIN_ENB, r);
}

void moveForward(int speedVal) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorPWM(speedVal, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorPWM(speedVal, speedVal);
}

void spinLeft(int speedVal) {
  // Left wheels reverse, Right wheels forward (on-the-spot left turn)
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorPWM(speedVal, speedVal);
}

void spinRight(int speedVal) {
  // Left wheels forward, Right wheels reverse (on-the-spot right turn)
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  setMotorPWM(speedVal, speedVal);
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// ============================================================================
// 5-ANGLE 230° SERVO RADAR SCAN & PATH DECISION
// ============================================================================
void scanAndAvoidObstacle() {
  stopMotors();
  delay(150);

  // Back up slightly to create turning clearance from the obstacle
  Serial.println(F("[AVOID] Backing up for clearance..."));
  moveBackward(SPEED_REVERSE);
  delay(240);
  stopMotors();
  delay(150);

  // Scan 5 angles across the 230° servo arc
  int scanAngles[5] = {ANGLE_FAR_RIGHT, ANGLE_RIGHT, ANGLE_CENTER, ANGLE_LEFT, ANGLE_FAR_LEFT};
  int distAtAngle[5];

  Serial.print(F("[RADAR SCAN] "));
  for (int i = 0; i < 5; i++) {
    writeServo230(scanAngles[i]);
    delay(260); // Wait for servo to reach target angle
    distAtAngle[i] = getFilteredDistanceCm();
    Serial.print(scanAngles[i]);
    Serial.print(F("deg="));
    Serial.print(distAtAngle[i]);
    Serial.print(F("cm  "));
  }
  Serial.println();

  // Return servo to dead-ahead center (115°)
  writeServo230(ANGLE_CENTER);
  delay(220);

  int distFarRight = distAtAngle[0];
  int distRight    = distAtAngle[1];
  int distLeft     = distAtAngle[3];
  int distFarLeft  = distAtAngle[4];

  int bestRight = max(distRight, distFarRight);
  int bestLeft  = max(distLeft, distFarLeft);

  // If all sides are blocked, execute a 180-degree U-turn
  if (bestRight < MIN_CLEAR_PATH_CM && bestLeft < MIN_CLEAR_PATH_CM) {
    Serial.println(F("[DECISION] Boxed in! Executing 180-deg U-turn..."));
    moveBackward(SPEED_REVERSE);
    delay(300);
    spinLeft(SPEED_TURN);
    delay(680);
    stopMotors();
    delay(150);
    return;
  }

  // Turn toward the side with greater clearance
  if (bestRight >= bestLeft) {
    if (distFarRight > distRight + 15) {
      Serial.println(F("[DECISION] Turning WIDE RIGHT (Far-Right clearest)"));
      spinRight(SPEED_TURN);
      delay(480);
    } else {
      Serial.println(F("[DECISION] Turning RIGHT"));
      spinRight(SPEED_TURN);
      delay(340);
    }
  } else {
    if (distFarLeft > distLeft + 15) {
      Serial.println(F("[DECISION] Turning WIDE LEFT (Far-Left clearest)"));
      spinLeft(SPEED_TURN);
      delay(480);
    } else {
      Serial.println(F("[DECISION] Turning LEFT"));
      spinLeft(SPEED_TURN);
      delay(340);
    }
  }

  stopMotors();
  delay(120);
}

// ============================================================================
// SETUP & PHASE 1 SELF-TEST
// ============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=========================================================="));
  Serial.println(F(" PHASE 1: ULTRASONIC + 230-DEG SERVO OBSTACLE AVOIDANCE"));
  Serial.println(F("=========================================================="));

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  radarServo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);

  // 1. Center servo at 115° (1500 us) so user can verify ultrasonic alignment
  Serial.println(F("[SELF-TEST 1/3] Centering 230-deg Servo at 115 deg (1500 us)..."));
  writeServo230(ANGLE_CENTER);
  delay(1500);

  // 2. Sweep test Left & Right
  Serial.println(F("[SELF-TEST 2/3] Sweeping Servo Right (65 deg) -> Left (165 deg) -> Center (115 deg)..."));
  writeServo230(ANGLE_RIGHT);
  delay(500);
  writeServo230(ANGLE_LEFT);
  delay(500);
  writeServo230(ANGLE_CENTER);
  delay(500);

  // 3. Ultrasonic distance test
  int initialDist = getFilteredDistanceCm();
  Serial.print(F("[SELF-TEST 3/3] Initial Ultrasonic Front Distance: "));
  Serial.print(initialDist);
  Serial.println(F(" cm"));
  Serial.println(F("[PHASE 1 READY] Starting autonomous navigation in 1 second..."));
  delay(1000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  int frontDistCm = getFilteredDistanceCm();

  Serial.print(F("Front Dist: "));
  Serial.print(frontDistCm);
  Serial.println(F(" cm"));

  if (frontDistCm <= STOP_DISTANCE_CM) {
    // Obstacle detected ahead -> Stop, scan with 230° servo, and turn
    scanAndAvoidObstacle();
  } else if (frontDistCm <= SLOW_DISTANCE_CM) {
    // Obstacle at medium range -> Drive cautiously at approach speed
    moveForward(SPEED_APPROACH);
  } else {
    // Clear road ahead -> Cruise at normal speed
    moveForward(SPEED_CRUISE);
  }

  delay(40);
}
