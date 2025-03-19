#include <EEPROM.h>

// ------------------- Constants -------------------

#define DIR_PIN 8
#define PWM_PIN 9
#define ENCODER_A 3
#define ENCODER_B 4

#define OPEN_BTN 5
#define CLOSE_BTN 6
#define HOME_SENSOR 11
#define PIR_SENSOR 7

#define DOOR_OPEN_POS 1000
#define DOOR_CLOSE_POS 0
#define DOOR_SPEED 255

#define ACCEL_STEP 10
#define AUTO_CLOSE_DELAY 5000
#define POS_SAVE_INTERVAL 300000 // 5 minutes

// ------------------- Variables -------------------

volatile long encoderCount = 0;
int currentSpeed = 0;
int targetSpeed = 0;
bool dir = true;

enum DoorState { IDLE, OPENING, CLOSING, HOMING };
DoorState doorState = HOMING;

unsigned long autoCloseTimer = 0;
unsigned long positionSaveTimer = 0;
long lastSavedCount = 0;

// ------------------- Encoder ISR -------------------

void encoderISR() {
  static uint8_t lastState = 0;
  uint8_t newState = digitalRead(ENCODER_A) | (digitalRead(ENCODER_B) << 1); // 🔧 FIXED
  int8_t delta = 0;

  switch (lastState) {
    case 0: if (newState == 1) delta = +1; else if (newState == 2) delta = -1; break;
    case 1: if (newState == 3) delta = +1; else if (newState == 0) delta = -1; break;
    case 3: if (newState == 2) delta = +1; else if (newState == 1) delta = -1; break;
    case 2: if (newState == 0) delta = +1; else if (newState == 3) delta = -1; break;
  }

  encoderCount += delta;
  lastState = newState;
}

// ------------------- Debug Print -------------------

void debugPrint(String msg) {
  Serial.println("[DEBUG] " + msg);
}

// ------------------- Emergency Stop -------------------

void emergencyStop() {
  currentSpeed = 0;
  targetSpeed = 0;
  analogWrite(PWM_PIN, 0);
  debugPrint("Emergency stop activated!");
}

// ------------------- Motor Control -------------------

void moveMotor(bool direction, int speed) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, speed);
}

// ------------------- Check for Motor Stuck -------------------

bool isStuck(long *lastPos, unsigned long *lastChange) {
  if (millis() - *lastChange > 2000) {
    if (abs(encoderCount - *lastPos) < 5) {
      return true;
    } else {
      *lastPos = encoderCount;
      *lastChange = millis();
    }
  }
  return false;
}

// ------------------- Setup -------------------

void setup() {
  Serial.begin(9600);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  encoderCount = EEPROM.get(0, encoderCount);
  lastSavedCount = encoderCount;

  debugPrint("System Initialized");
}

// ------------------- Main Loop -------------------

void loop() {
  static long lastPos = encoderCount;
  static unsigned long lastChange = millis();

  // -------- HOMING LOGIC --------
  if (doorState == HOMING) {
    moveMotor(true, DOOR_SPEED);
    if (digitalRead(HOME_SENSOR) == LOW) {
      emergencyStop();
      encoderCount = 0;
      EEPROM.put(0, encoderCount);
      lastSavedCount = 0;
      debugPrint("Homed! Encoder = 0");
      doorState = IDLE;
      autoCloseTimer = millis();
    }
    return;
  }

  // -------- BUTTON CONTROLS --------
  if (doorState == IDLE) {
    if (digitalRead(OPEN_BTN) == LOW) {
      doorState = OPENING;
      autoCloseTimer = millis(); // reset timer
    } else if (digitalRead(CLOSE_BTN) == LOW) {
      doorState = CLOSING;
    }
  }

  // Emergency Stop During Motion
  if (doorState == OPENING && digitalRead(CLOSE_BTN) == LOW) {
    emergencyStop();
    doorState = IDLE;
    return;
  }

  if (doorState == CLOSING && digitalRead(OPEN_BTN) == LOW) {
    emergencyStop();
    doorState = OPENING;
    return;
  }

  // -------- PIR SENSOR --------
  if (digitalRead(PIR_SENSOR) == LOW && doorState == IDLE) {
    doorState = OPENING;
    autoCloseTimer = millis();
  }

  // -------- DOOR ACTION --------
  if (doorState == OPENING) {
    dir = true;
    targetSpeed = DOOR_SPEED;
    if (encoderCount >= DOOR_OPEN_POS) {
      emergencyStop();
      doorState = IDLE;
      autoCloseTimer = millis();
    }
  } else if (doorState == CLOSING) {
    dir = false;
    targetSpeed = DOOR_SPEED;
    if (encoderCount <= DOOR_CLOSE_POS) {
      emergencyStop();
      doorState = IDLE;
    }
  } else {
    targetSpeed = 0;
  }

  // -------- STUCK CHECK --------
  if (doorState != IDLE && isStuck(&lastPos, &lastChange)) {
    debugPrint("⚠️ Motor stuck detected!");
    emergencyStop();
    doorState = IDLE;
    // Optional: Add alert system here
    return;
  }

  // -------- ACCEL / DECEL --------
  if (currentSpeed < targetSpeed) currentSpeed += ACCEL_STEP;
  else if (currentSpeed > targetSpeed) currentSpeed -= ACCEL_STEP;

  currentSpeed = constrain(currentSpeed, 0, 255);
  moveMotor(dir, currentSpeed);

  // -------- AUTO CLOSE --------
  if (doorState == IDLE && encoderCount >= DOOR_OPEN_POS) {
    if (millis() - autoCloseTimer > AUTO_CLOSE_DELAY) {
      doorState = CLOSING;
    }
  }

  // -------- EEPROM SAVE --------
  if (millis() - positionSaveTimer > POS_SAVE_INTERVAL) {
    if (abs(encoderCount - lastSavedCount) > 5) { // Save only if changed
      EEPROM.put(0, encoderCount);
      lastSavedCount = encoderCount;
      debugPrint("Position saved to EEPROM: " + String(encoderCount));
    }
    positionSaveTimer = millis();
  }

  delay(10); // Consider replacing with non-blocking loop later
}
