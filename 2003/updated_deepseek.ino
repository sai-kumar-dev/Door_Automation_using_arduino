/*
 * Smart Door Controller with Enhanced Safety Features
 * Final Version 3.1
 */

#include <EEPROM.h>

// ========== HARDWARE CONFIG ==========
#define DIR_PIN         8
#define PWM_PIN         9
#define ENCODER_A       3
#define ENCODER_B       4
#define HOME_SENSOR     11
#define PIR_SENSOR      12
#define OPEN_BTN        5
#define CLOSE_BTN       6

// ========== DEBUG CONFIG ==========
#define DEBUG           true
#define SERIAL_BAUD     9600

// ========== PARAMETERS ==========
#define MAX_SPEED       180
#define MIN_SPEED       50
#define AUTO_CLOSE_DELAY 5000UL
#define DEBOUNCE_DELAY  200UL
#define MAX_MOVE_TIME   8000UL
#define POS_SAVE_INTERVAL 300000UL
#define STUCK_TIMEOUT   1000UL
#define STUCK_TOLERANCE 3
#define SAFETY_MARGIN   50
#define HOMING_INTERVAL 86400000UL

// ========== STATE VARIABLES ==========
volatile long encoderCount = 0;
long openPosition = 0;
long closedPosition = -1050;
bool isHomed = false;

unsigned long autoCloseTimer = 0;
unsigned long positionSaveTimer = 0;
unsigned long lastHomingTime = 0;
unsigned long moveStartTime = 0;

enum DoorState { IDLE, OPENING, CLOSING, HOMING, STOPPED };
DoorState doorState = HOMING;

// ========== STRUCT ==========
struct PersistentData {
  long openPos;
  long closedPos;
  bool calibrated;
};

// ========== DEBUG FUNCTION ==========
void debugPrint(String msg) {
  if (DEBUG) Serial.println(msg);
}

// ========== ENCODER ISR ==========
void updateEncoder() {
  static uint8_t lastState = 0;
  uint8_t newState = digitalRead(ENCODER_A) | (digitalRead(ENCODER_B) << 1);
  if (newState == lastState) return;
  const int8_t transitions[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  encoderCount += transitions[(lastState << 2) | newState];
  lastState = newState;
}

// ========== EEPROM ==========
void savePosition() {
  PersistentData data = {openPosition, closedPosition, true};
  EEPROM.put(0, data);
  debugPrint("[EEPROM] Position saved");
}

void loadPosition() {
  PersistentData data;
  EEPROM.get(0, data);
  if (data.calibrated) {
    openPosition = data.openPos;
    closedPosition = data.closedPos;
    encoderCount = openPosition;
    debugPrint("[EEPROM] Loaded saved position");
  } else {
    debugPrint("[EEPROM] No calibration data found");
  }
}

// ========== MOTOR CONTROL ==========
void stopMotor() {
  analogWrite(PWM_PIN, 0);
  debugPrint("[MOTOR] STOP");
}

void setPwmFrequency() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x05;  // 31.25Hz
}

// ========== SAFETY ==========
void emergencyStop() {
  stopMotor();
  doorState = STOPPED;
  savePosition();
  debugPrint("[SAFETY] Emergency Stop Triggered!");
}

void checkSoftwareEndstops() {
  if (encoderCount > openPosition + SAFETY_MARGIN || encoderCount < closedPosition - SAFETY_MARGIN) {
    debugPrint("[SAFETY] Software endstop breach");
    emergencyStop();
  }
}

// ========== HOMING ==========
void homeDoor() {
  debugPrint("[HOMING] Starting...");
  doorState = HOMING;
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 100);
  unsigned long startTime = millis();

  while (digitalRead(HOME_SENSOR) == HIGH) {
    if (millis() - startTime > 5000) {
      debugPrint("[ERROR] Homing Timeout");
      emergencyStop();
      return;
    }
    if (checkButton(OPEN_BTN)) {
      debugPrint("[HOMING] Aborted by User");
      emergencyStop();
      return;
    }
    delay(10);
  }

  stopMotor();
  encoderCount = openPosition;
  isHomed = true;
  doorState = IDLE;
  lastHomingTime = millis();
  debugPrint("[HOMING] Completed");
}

// ========== CALIBRATION ==========
void enterCalibration() {
  debugPrint("[CALIB] Mode Activated");

  debugPrint("[CALIB] Move to OPEN pos & hold Open button");
  while (digitalRead(OPEN_BTN) == HIGH) {
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 100);
    delay(10);
  }
  stopMotor();
  openPosition = encoderCount;

  debugPrint("[CALIB] Move to CLOSED pos & hold Close button");
  while (digitalRead(CLOSE_BTN) == HIGH) {
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 100);
    delay(10);
  }
  stopMotor();
  closedPosition = encoderCount;

  savePosition();
  debugPrint("[CALIB] Complete");
}

// ========== BUTTON DEBOUNCE ==========
bool checkButton(int pin) {
  static unsigned long lastPress[14] = {0};
  if (digitalRead(pin) == LOW && millis() - lastPress[pin] > DEBOUNCE_DELAY) {
    lastPress[pin] = millis();
    return true;
  }
  return false;
}

// ========== STUCK DETECTION ==========
bool isStuck(long* lastPos, unsigned long* lastChange) {
  if (abs(encoderCount - *lastPos) > STUCK_TOLERANCE) {
    *lastPos = encoderCount;
    *lastChange = millis();
    return false;
  }
  return (millis() - *lastChange) > STUCK_TIMEOUT;
}

// ========== MOVEMENT ==========
void moveDoor(bool direction, long target) {
  digitalWrite(DIR_PIN, direction);
  moveStartTime = millis();
  long startPos = encoderCount;
  long lastPos = encoderCount;
  unsigned long lastChange = millis();

  while (millis() - moveStartTime < MAX_MOVE_TIME) {
    if ((direction == HIGH && encoderCount >= target) ||
        (direction == LOW && encoderCount <= target)) break;

    long remaining = abs(target - encoderCount);
    int speed = map(remaining, 0, abs(target - startPos), MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));

    if (isStuck(&lastPos, &lastChange)) {
      debugPrint("[ERROR] Motor Stuck");
      emergencyStop();
      return;
    }

    checkSoftwareEndstops();
    delay(10);
  }

  stopMotor();
  debugPrint("[MOVE] Door Reached Target");
}

// ========== STATE HANDLING ==========
void handleIdleState() {
  if (checkButton(OPEN_BTN) || digitalRead(PIR_SENSOR) == LOW) {
    debugPrint("[STATE] Opening...");
    doorState = OPENING;
    autoCloseTimer = millis();
  }
  else if (checkButton(CLOSE_BTN)) {
    debugPrint("[STATE] Closing...");
    doorState = CLOSING;
  }
  else if (millis() - autoCloseTimer > AUTO_CLOSE_DELAY) {
    debugPrint("[AUTO] Closing after delay");
    doorState = CLOSING;
  }
}

void handleMovement() {
  if (doorState == OPENING) {
    moveDoor(HIGH, openPosition);
    doorState = IDLE;
    autoCloseTimer = millis();
  } else if (doorState == CLOSING) {
    moveDoor(LOW, closedPosition);
    doorState = IDLE;
  }
}

void handleStoppedState() {
  if (checkButton(OPEN_BTN) || checkButton(CLOSE_BTN)) {
    debugPrint("[SYSTEM] Resuming from STOP");
    doorState = IDLE;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(SERIAL_BAUD);
  setPwmFrequency();

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);

  if (digitalRead(OPEN_BTN) == LOW && digitalRead(CLOSE_BTN) == LOW) {
    enterCalibration();
  }

  loadPosition();
  homeDoor();
}

// ========== LOOP ==========
void loop() {
  static unsigned long lastMaintenance = 0;

  if (millis() - lastMaintenance > 60000) {
    lastMaintenance = millis();
    if (millis() - positionSaveTimer > POS_SAVE_INTERVAL) {
      savePosition();
      positionSaveTimer = millis();
    }
    if (millis() - lastHomingTime > HOMING_INTERVAL) {
      debugPrint("[MAINT] Rehoming recommended");
    }
  }

  switch (doorState) {
    case IDLE:    handleIdleState(); break;
    case OPENING:
    case CLOSING: handleMovement(); break;
    case STOPPED: handleStoppedState(); break;
    case HOMING:  break; // Already handled
  }
}
