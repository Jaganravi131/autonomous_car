/*
 * ============================================================================
 * PHASE 2: 4WD AUTONOMOUS CAR — ULTRASONIC + 230° SERVO + 2x IR SENSOR FUSION
 * ============================================================================
 * Run this ONLY after Phase 1 (Ultrasonic + Servo + Motors) passes!
 * Keep all Phase 1 wires untouched and add the 2x IR sensors:
 *   - Left IR Sensor OUT  -> Arduino Pin A0
 *   - Right IR Sensor OUT -> Arduino Pin A1
 *   - Both IR VCC         -> +5.0V Rail (from Buck Converter)
 *   - Both IR GND         -> Common GND
 *
 * Full pin map (identical to Phase 1, plus the two IR sensors):
 *   - D5  -> L298N ENA (Left Motors Speed PWM - Timer0)
 *   - D6  -> L298N ENB (Right Motors Speed PWM - Timer0)
 *   - D7  -> 230° Servo SIGNAL
 *   - D8  -> L298N IN1 (Left Motors Forward)
 *   - D9  -> L298N IN2 (Left Motors Backward)
 *   - D10 -> L298N IN3 (Right Motors Forward)
 *   - D11 -> L298N IN4 (Right Motors Backward)
 *   - D12 -> HC-SR04 TRIG
 *   - D13 <- HC-SR04 ECHO
 *
 * Supports Two Selectable Modes (change OPERATING_MODE below):
 *   1 = MODE_OBSTACLE_FUSION (Default):
 *       IR sensors face forward-left & forward-right at ~35° angles to protect
 *       wheel corners & blind spots (chair legs, angled walls), fused with
 *       HC-SR04 Ultrasonic + 230° Servo long-range radar scanning.
 *   2 = MODE_LINE_AND_OBSTACLE:
 *       IR sensors point downward (1.5 cm above floor) to follow a black line
 *       track while HC-SR04 Ultrasonic stops/avoids obstacles on the track.
 * ============================================================================
 */

#include <Servo.h>

// -------------------- SELECT OPERATING MODE ------------------------
#define MODE_OBSTACLE_FUSION    1
#define MODE_LINE_AND_OBSTACLE  2
#define OPERATING_MODE          MODE_OBSTACLE_FUSION

// -------------------- MOTOR DRIVER PINS (L298N) --------------------
const int PIN_IN1 = 8;    // Left motors forward
const int PIN_IN2 = 9;    // Left motors backward
const int PIN_IN3 = 10;   // Right motors forward
const int PIN_IN4 = 11;   // Right motors backward
const int PIN_ENA = 5;    // Left motors PWM speed  (Timer0)
const int PIN_ENB = 6;    // Right motors PWM speed (Timer0)

// -------------------- ULTRASONIC & 230° SERVO PINS -----------------
const int PIN_TRIG  = 12;
const int PIN_ECHO  = 13;
const int PIN_SERVO = 7;

// -------------------- 2x IR SENSOR PINS (ADDED IN PHASE 2) ---------
const int PIN_IR_LEFT  = A0;
const int PIN_IR_RIGHT = A1;

// Standard LM393 IR obstacle modules output LOW (0) when an obstacle reflects IR light
const int IR_OBSTACLE_DETECTED = LOW;

// -------------------- 230-DEGREE SERVO CALIBRATION -----------------
const int SERVO_MIN_US       = 500;
const int SERVO_MAX_US       = 2500;
const int ANGLE_FAR_RIGHT    = 30;
const int ANGLE_RIGHT        = 65;
const int ANGLE_CENTER       = 115;  // Dead-ahead center (1500 us)
const int ANGLE_LEFT         = 165;
const int ANGLE_FAR_LEFT     = 200;

// -------------------- THRESHOLDS & SPEEDS --------------------------
const int STOP_DISTANCE_CM   = 28;
const int SLOW_DISTANCE_CM   = 45;
const int MIN_CLEAR_PATH_CM  = 22;
const int MAX_SENSOR_DIST_CM = 300;

const int SPEED_CRUISE       = 165;
const int SPEED_APPROACH     = 120;
const int SPEED_LINE_FOLLOW  = 135;
const int SPEED_REVERSE      = 150;
const int SPEED_TURN         = 190;

Servo radarServo;

// ============================================================================
// HELPER: Move 230-Degree Servo to Physical Angle (0° to 230°)
// ============================================================================
void writeServo230(int physicalAngle) {
  physicalAngle = constrain(physicalAngle, 0, 230);
  long pulseUs = map(physicalAngle, 0, 230, SERVO_MIN_US, SERVO_MAX_US);
  radarServo.writeMicroseconds((int)pulseUs);
}

// ============================================================================
// HELPER: Ultrasonic Distance Measurement (cm)
// ============================================================================
int readSinglePingCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 22000UL);
  if (duration == 0) return MAX_SENSOR_DIST_CM;
  int cm = (int)(duration * 0.0343 / 2.0);
  return constrain(cm, 2, MAX_SENSOR_DIST_CM);
}

int getFilteredDistanceCm() {
  int r1 = readSinglePingCm();
  delay(8);
  int r2 = readSinglePingCm();
  return (r1 + r2) / 2;
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================
void setMotorPWM(int leftSpeed, int rightSpeed) {
  analogWrite(PIN_ENA, constrain(leftSpeed, 0, 255));
  analogWrite(PIN_ENB, constrain(rightSpeed, 0, 255));
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
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  setMotorPWM(speedVal, speedVal);
}

void spinRight(int speedVal) {
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
// 5-ANGLE 230° SERVO RADAR SCAN & PATH SELECTION
// ============================================================================
void scanAndAvoidObstacle() {
  stopMotors();
  delay(120);

  moveBackward(SPEED_REVERSE);
  delay(240);
  stopMotors();
  delay(120);

  int scanAngles[5] = {ANGLE_FAR_RIGHT, ANGLE_RIGHT, ANGLE_CENTER, ANGLE_LEFT, ANGLE_FAR_LEFT};
  int distAtAngle[5];

  for (int i = 0; i < 5; i++) {
    writeServo230(scanAngles[i]);
    delay(240);
    distAtAngle[i] = getFilteredDistanceCm();
  }

  writeServo230(ANGLE_CENTER);
  delay(200);

  int bestRight = max(distAtAngle[0], distAtAngle[1]);
  int bestLeft  = max(distAtAngle[3], distAtAngle[4]);

  // Also check if an IR corner sensor is currently blocked before turning that way
  bool irLeftBlocked  = (digitalRead(PIN_IR_LEFT)  == IR_OBSTACLE_DETECTED);
  bool irRightBlocked = (digitalRead(PIN_IR_RIGHT) == IR_OBSTACLE_DETECTED);
  if (irLeftBlocked)  bestLeft  = 0;
  if (irRightBlocked) bestRight = 0;

  if (bestRight < MIN_CLEAR_PATH_CM && bestLeft < MIN_CLEAR_PATH_CM) {
    Serial.println(F("[FUSION] All front/side paths blocked -> U-Turn"));
    moveBackward(SPEED_REVERSE);
    delay(300);
    spinLeft(SPEED_TURN);
    delay(660);
  } else if (bestRight >= bestLeft) {
    Serial.println(F("[FUSION] Turning RIGHT toward open space"));
    spinRight(SPEED_TURN);
    delay(distAtAngle[0] > distAtAngle[1] + 15 ? 460 : 330);
  } else {
    Serial.println(F("[FUSION] Turning LEFT toward open space"));
    spinLeft(SPEED_TURN);
    delay(distAtAngle[4] > distAtAngle[3] + 15 ? 460 : 330);
  }

  stopMotors();
  delay(100);
}

// ============================================================================
// MODE 1: MULTI-SENSOR OBSTACLE AVOIDANCE (ULTRASONIC + 2x IR CORNER GUARD)
// ============================================================================
void runObstacleFusionMode() {
  bool leftIrHit  = (digitalRead(PIN_IR_LEFT)  == IR_OBSTACLE_DETECTED);
  bool rightIrHit = (digitalRead(PIN_IR_RIGHT) == IR_OBSTACLE_DETECTED);
  int frontDistCm = getFilteredDistanceCm();

  Serial.print(F("US: "));
  Serial.print(frontDistCm);
  Serial.print(F("cm | IR_L: "));
  Serial.print(leftIrHit ? F("HIT") : F("CLR"));
  Serial.print(F(" | IR_R: "));
  Serial.println(rightIrHit ? F("HIT") : F("CLR"));

  // Priority 1: Both IR sensors hit OR Ultrasonic front obstacle too close
  if ((leftIrHit && rightIrHit) || frontDistCm <= STOP_DISTANCE_CM) {
    scanAndAvoidObstacle();
    return;
  }

  // Priority 2: Left corner IR sensor hit -> Instant reflex dodge to the Right
  if (leftIrHit) {
    Serial.println(F("[IR REFLEX] Left corner obstacle! Dodging Right..."));
    moveBackward(SPEED_REVERSE);
    delay(140);
    spinRight(SPEED_TURN);
    delay(240);
    stopMotors();
    return;
  }

  // Priority 3: Right corner IR sensor hit -> Instant reflex dodge to the Left
  if (rightIrHit) {
    Serial.println(F("[IR REFLEX] Right corner obstacle! Dodging Left..."));
    moveBackward(SPEED_REVERSE);
    delay(140);
    spinLeft(SPEED_TURN);
    delay(240);
    stopMotors();
    return;
  }

  // Priority 4: Clear corners -> Drive forward with adaptive ultrasonic speed
  if (frontDistCm <= SLOW_DISTANCE_CM) {
    moveForward(SPEED_APPROACH);
  } else {
    moveForward(SPEED_CRUISE);
  }
}

// ============================================================================
// MODE 2: LINE FOLLOWER + ULTRASONIC OBSTACLE STOP
// ============================================================================
void runLineAndObstacleMode() {
  int frontDistCm = readSinglePingCm();
  if (frontDistCm <= STOP_DISTANCE_CM) {
    stopMotors();
    Serial.println(F("[LINE MODE] Obstacle on track! Waiting or scanning..."));
    delay(200);
    return;
  }

  // On a white floor with black line:
  // White floor reflects IR (LOW), Black line absorbs IR (HIGH)
  bool leftOnBlack  = (digitalRead(PIN_IR_LEFT)  == HIGH);
  bool rightOnBlack = (digitalRead(PIN_IR_RIGHT) == HIGH);

  if (!leftOnBlack && !rightOnBlack) {
    moveForward(SPEED_LINE_FOLLOW);
  } else if (leftOnBlack && !rightOnBlack) {
    spinLeft(SPEED_TURN);
  } else if (!leftOnBlack && rightOnBlack) {
    spinRight(SPEED_TURN);
  } else {
    stopMotors(); // Reached stop line / intersection
  }
}

// ============================================================================
// SETUP & PHASE 2 DIAGNOSTIC CHECK
// ============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("=========================================================="));
  Serial.println(F(" PHASE 2: ULTRASONIC + 230° SERVO + 2x IR SENSOR FUSION"));
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
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  radarServo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  writeServo230(ANGLE_CENTER);
  delay(1000);

  // Print initial IR and Ultrasonic states to verify Phase 2 wiring
  Serial.print(F("[PHASE 2 CHECK] Left IR (A0)="));
  Serial.print(digitalRead(PIN_IR_LEFT) == IR_OBSTACLE_DETECTED ? F("DETECTED") : F("CLEAR"));
  Serial.print(F(" | Right IR (A1)="));
  Serial.print(digitalRead(PIN_IR_RIGHT) == IR_OBSTACLE_DETECTED ? F("DETECTED") : F("CLEAR"));
  Serial.print(F(" | Front US="));
  Serial.print(getFilteredDistanceCm());
  Serial.println(F(" cm"));
  delay(800);
}

void loop() {
  if (OPERATING_MODE == MODE_OBSTACLE_FUSION) {
    runObstacleFusionMode();
  } else {
    runLineAndObstacleMode();
  }
  delay(25);
}
