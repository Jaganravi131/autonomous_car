/*
 * ============================================================================
 * PHASE 3 (PART A): ARDUINO UNO — ML ACTUATOR + HARDWARE SAFETY REFLEX CONTROLLER
 * ============================================================================
 * Hardware Connections (Phase 1 + Phase 2 + ESP32-CAM UART Bridge):
 *   - D2..D7  -> L298N Motor Driver (IN1=D2, IN2=D3, IN3=D4, ENA=D5, ENB=D6, IN4=D7)
 *   - D8, D9  -> HC-SR04 Ultrasonic (TRIG=D8, ECHO=D9)
 *   - D10     -> 230° Servo Motor Signal
 *   - A0, A1  -> Left IR Sensor (A0), Right IR Sensor (A1)
 *   - A2      -> ESP32-CAM U0T (GPIO1 TX) [1 Direct Female-to-Male Jumper Wire]
 *   - A3      -> ESP32-CAM U0R (GPIO3 RX) [Optional: via 1k/2k divider for telemetry]
 *   - GND     -> Common GND shared with ESP32-CAM, Buck Converter, and L298N
 *
 * Supported Commands from ESP32-CAM (9600 baud over A2/A3 or USB Serial):
 *   'F' = Forward Straight
 *   'G' = Forward-Left Smooth Curve  (ML gentle left steering)
 *   'I' = Forward-Right Smooth Curve (ML gentle right steering)
 *   'L' = Spin Left (Sharp turn)
 *   'R' = Spin Right (Sharp turn)
 *   'B' = Backward
 *   'S' = Stop
 *   '1'..'9' = Set base speed (1=110 PWM ... 9=245 PWM)
 *   'C' = Camera / ML Control Mode (with Ultrasonic + IR Safety Override)
 *   'A' = Full On-Board Sensor Autonomous Fallback Mode (Phase 2 logic)
 * ============================================================================
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// -------------------- MOTOR DRIVER PINS (L298N) --------------------
const int PIN_IN1 = 2;
const int PIN_IN2 = 3;
const int PIN_IN3 = 4;
const int PIN_ENA = 5;
const int PIN_ENB = 6;
const int PIN_IN4 = 7;

// -------------------- SENSORS & 230° SERVO PINS --------------------
const int PIN_TRIG     = 8;
const int PIN_ECHO     = 9;
const int PIN_SERVO    = 10;
const int PIN_IR_LEFT  = A0;
const int PIN_IR_RIGHT = A1;

// -------------------- ESP32-CAM UART BRIDGE PINS -------------------
const int PIN_ESP_RX   = A2; // Connects to ESP32-CAM U0T (GPIO1 TX)
const int PIN_ESP_TX   = A3; // Optional telemetry -> ESP32-CAM IO13 via 1k/2k divider
                             // (NOT U0R/GPIO3, so the flashing pins stay free)

SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX); // RX, TX
Servo radarServo;

// -------------------- CONSTANTS & SAFETY SETTINGS ------------------
const int SERVO_MIN_US           = 500;
const int SERVO_MAX_US           = 2500;
const int ANGLE_CENTER           = 115; // 230° servo dead-ahead center
const int IR_OBSTACLE_DETECTED   = LOW;

const int EMERGENCY_STOP_CM      = 20;  // Hard safety override distance in ML mode
const unsigned long WATCHDOG_MS  = 900; // Stop motors if no ML command for 900 ms

// ---- ESP32-CAM link protection --------------------------------------------
// The ESP32 prints its own boot logs and debug text on the SAME wire it uses
// for commands, and words like "[CAM] Streaming..." contain M, A, C and S.
// Commands are therefore wrapped in a frame:  '~' + letter + '\n'.
const char CAM_FRAME_CHAR        = '~';
const unsigned long CAM_GRACE_MS = 3000; // ignore cam input for 3 s after boot

// -------------------- RUNTIME STATE --------------------------------
bool autonomousSensorMode        = false; // false = Camera/ML Mode ('C'), true = Sensor Autonomous ('A')
char currentCommand              = 'S';
int baseSpeedPWM                 = 165;
unsigned long lastCommandTimeMs  = 0;
unsigned long lastTelemetryMs    = 0;

// ============================================================================
// HELPER: 230° Servo & Ultrasonic Sensor
// ============================================================================
void writeServo230(int physicalAngle) {
  physicalAngle = constrain(physicalAngle, 0, 230);
  long pulseUs = map(physicalAngle, 0, 230, SERVO_MIN_US, SERVO_MAX_US);
  radarServo.writeMicroseconds((int)pulseUs);
}

int readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 18000UL);
  if (duration == 0) return 300;
  return constrain((int)(duration * 0.0343 / 2.0), 2, 300);
}

// ============================================================================
// MOTOR ACTUATION PRIMITIVES
// ============================================================================
void driveMotors(int leftPWM, int rightPWM, bool leftFwd, bool rightFwd) {
  digitalWrite(PIN_IN1, leftFwd ? HIGH : LOW);
  digitalWrite(PIN_IN2, leftFwd ? LOW  : HIGH);
  digitalWrite(PIN_IN3, rightFwd ? HIGH : LOW);
  digitalWrite(PIN_IN4, rightFwd ? LOW  : HIGH);
  analogWrite(PIN_ENA, constrain(leftPWM, 0, 255));
  analogWrite(PIN_ENB, constrain(rightPWM, 0, 255));
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
// EXECUTE DRIVING COMMAND WITH HARDWARE SAFETY REFLEX OVERRIDE
// ============================================================================
void executeCommandWithSafety(char cmd, int frontDistCm, bool irLeftHit, bool irRightHit) {
  bool isForwardMotion = (cmd == 'F' || cmd == 'G' || cmd == 'I');

  // HARDWARE SAFETY REFLEX LAYER:
  // If ML commands forward motion into a wall or corner obstacle, override immediately!
  if (isForwardMotion) {
    if (frontDistCm <= EMERGENCY_STOP_CM || (irLeftHit && irRightHit)) {
      stopMotors();
      Serial.println(F("[SAFETY OVERRIDE] Front obstacle < 20cm! Blocking forward command."));
      return;
    }
    if (irLeftHit) {
      // Gently veer right away from left-corner obstacle
      driveMotors(baseSpeedPWM + 25, baseSpeedPWM / 3, true, true);
      return;
    }
    if (irRightHit) {
      // Gently veer left away from right-corner obstacle
      driveMotors(baseSpeedPWM / 3, baseSpeedPWM + 25, true, true);
      return;
    }
  }

  switch (cmd) {
    case 'F': // Forward Straight
      driveMotors(baseSpeedPWM, baseSpeedPWM, true, true);
      break;

    case 'G': // Forward-Left Curve (Smooth ML Steering Left)
      driveMotors((int)(baseSpeedPWM * 0.42), baseSpeedPWM, true, true);
      break;

    case 'I': // Forward-Right Curve (Smooth ML Steering Right)
      driveMotors(baseSpeedPWM, (int)(baseSpeedPWM * 0.42), true, true);
      break;

    case 'L': // Sharp Spin Left
      driveMotors(constrain(baseSpeedPWM + 25, 0, 255),
                  constrain(baseSpeedPWM + 25, 0, 255), false, true);
      break;

    case 'R': // Sharp Spin Right
      driveMotors(constrain(baseSpeedPWM + 25, 0, 255),
                  constrain(baseSpeedPWM + 25, 0, 255), true, false);
      break;

    case 'B': // Backward
      driveMotors(baseSpeedPWM, baseSpeedPWM, false, false);
      break;

    case 'S': // Stop
    default:
      stopMotors();
      break;
  }
}

// ============================================================================
// PARSE INCOMING SERIAL COMMAND (FROM ESP32-CAM OR USB SERIAL)
// ============================================================================
void processIncomingByte(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;
  c = toupper(c);

  if (c >= '1' && c <= '9') {
    baseSpeedPWM = map(c - '1', 0, 8, 110, 245);
    Serial.print(F("[SPEED] Base PWM set to "));
    Serial.println(baseSpeedPWM);
    return;
  }

  if (c == 'A') {
    autonomousSensorMode = true;
    Serial.println(F("[MODE] Switched to On-Board Sensor Autonomous Mode (A)"));
    return;
  }

  if (c == 'C') {
    autonomousSensorMode = false;
    currentCommand = 'S';
    stopMotors();
    Serial.println(F("[MODE] Switched to ESP32-CAM / ML Control Mode (C)"));
    return;
  }

  if (c == 'F' || c == 'G' || c == 'I' || c == 'L' || c == 'R' || c == 'B' || c == 'S') {
    autonomousSensorMode = false;
    currentCommand = c;
    lastCommandTimeMs = millis();
  }
}

// ============================================================================
// FALLBACK ON-BOARD SENSOR AUTONOMOUS ROUTINE (IF MODE 'A' SELECTED)
// ============================================================================
void runFallbackSensorAutonomous(int frontDistCm, bool irLeftHit, bool irRightHit) {
  if (frontDistCm <= 28 || (irLeftHit && irRightHit)) {
    stopMotors();
    delay(100);
    driveMotors(150, 150, false, false);
    delay(220);
    stopMotors();

    writeServo230(65);
    delay(240);
    int rightDist = readDistanceCm();

    writeServo230(165);
    delay(280);
    int leftDist = readDistanceCm();

    writeServo230(ANGLE_CENTER);
    delay(180);

    if (rightDist >= leftDist) {
      driveMotors(190, 190, true, false);
    } else {
      driveMotors(190, 190, false, true);
    }
    delay(340);
    stopMotors();
  } else if (irLeftHit) {
    driveMotors(190, 190, true, false);
    delay(180);
  } else if (irRightHit) {
    driveMotors(190, 190, false, true);
    delay(180);
  } else {
    driveMotors(baseSpeedPWM, baseSpeedPWM, true, true);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

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

  Serial.println(F("=========================================================="));
  Serial.println(F(" PHASE 3: ARDUINO ML CONTROLLER + SAFETY REFLEX READY"));
  Serial.println(F(" Listening for ESP32-CAM commands on Pin A2 (9600 baud)"));
  Serial.println(F("=========================================================="));
  lastCommandTimeMs = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
// FRAMED ESP32-CAM INPUT:   '~' <command> '\n'
// Everything outside a frame (boot logs, "[CAM] ..." debug lines, noise) is
// ignored, so the ESP32 can never accidentally steer the car or change modes.
// ============================================================================
static bool camInFrame = false;

void handleCamByte(char c) {
  if (millis() < CAM_GRACE_MS) return;      // ignore the ESP32 boot-log burst

  if (c == CAM_FRAME_CHAR)   { camInFrame = true;  return; }
  if (c == '\n' || c == '\r') { camInFrame = false; return; }
  if (!camInFrame) return;                  // unframed noise -> ignore

  camInFrame = false;                       // exactly one command per frame
  processIncomingByte(c);
}

// ============================================================================
void loop() {
  // 1. Read commands from ESP32-CAM (Pin A2) - ALWAYS inside a frame.
  //    The ESP32 also prints boot logs and debug text on this same wire.
  //    Words like "[CAM] Streaming..." contain M, A, C and S, so unframed
  //    text must NEVER be treated as a command.  Frame format:  ~X\n
  while (espSerial.available() > 0) {
    handleCamByte((char)espSerial.read());
  }

  // 2. USB Serial Monitor for bench testing - accepts plain characters
  //    (this link is trusted: it is your own keyboard, no boot logs)
  while (Serial.available() > 0) {
    processIncomingByte((char)Serial.read());
  }

  // 3. Read safety sensors
  int frontDistCm = readDistanceCm();
  bool irLeftHit  = (digitalRead(PIN_IR_LEFT)  == IR_OBSTACLE_DETECTED);
  bool irRightHit = (digitalRead(PIN_IR_RIGHT) == IR_OBSTACLE_DETECTED);

  // 4. Actuate according to active mode
  if (autonomousSensorMode) {
    runFallbackSensorAutonomous(frontDistCm, irLeftHit, irRightHit);
  } else {
    // Watchdog: stop car if Wi-Fi/ML stream stops sending commands
    if (currentCommand != 'S' && (millis() - lastCommandTimeMs > WATCHDOG_MS)) {
      currentCommand = 'S';
      stopMotors();
      Serial.println(F("[WATCHDOG] Command timeout -> Motors stopped."));
    }
    executeCommandWithSafety(currentCommand, frontDistCm, irLeftHit, irRightHit);
  }

  // 5. Send periodic sensor telemetry every 250ms
  if (millis() - lastTelemetryMs >= 250) {
    lastTelemetryMs = millis();
    espSerial.print(F("D:"));
    espSerial.print(frontDistCm);
    espSerial.print(F(",L:"));
    espSerial.print(irLeftHit ? 1 : 0);
    espSerial.print(F(",R:"));
    espSerial.print(irRightHit ? 1 : 0);
    espSerial.print(F(",M:"));
    espSerial.println(currentCommand);
  }

  delay(15);
}
