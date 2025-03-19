/* 
 * Smart Door Controller with Enhanced Safety Features
 * Final Version 3.0
 * Features:
 * - Non-blocking state machine
 * - Encoder-based position tracking
 * - Automatic homing
 * - Software endstops
 * - Emergency stop
 * - EEPROM storage
 * - Stuck detection
 * - PWM optimization
 */

// ======== HARDWARE CONFIGURATION ========
#define DIR_PIN         8
#define PWM_PIN         9
#define ENCODER_A       3
#define ENCODER_B       4
#define HOME_SENSOR     11
#define PIR_SENSOR      12
#define OPEN_BTN        5
#define CLOSE_BTN       6

// ======== DEBUG CONFIGURATION ========
#define DEBUG           true
#define SERIAL_BAUD     9600

// ======== OPERATIONAL PARAMETERS ========
#define MAX_SPEED       180
#define MIN_SPEED       40
#define AUTO_CLOSE_DELAY 5000UL
#define DEBOUNCE_DELAY  250UL
#define MAX_MOVE_TIME   8000UL
#define POS_SAVE_INTERVAL 300000UL  // 5 minutes
#define STUCK_TIMEOUT   1000UL      // 1 second
#define SAFETY_MARGIN   50
#define HOMING_INTERVAL 86400000UL  // 24 hours

// ======== GLOBAL VARIABLES ========
volatile long encoderCount = 0;
long openPosition = 0;
long closedPosition = -1050;

unsigned long autoCloseTimer = 0;
unsigned long positionSaveTimer = 0;
unsigned long lastHomingTime = 0;

enum DoorState { IDLE, OPENING, CLOSING, HOMING, STOPPED };
DoorState doorState = HOMING;
bool isHomed = false;

// ======== ENCODER ISR ========
void updateEncoder() {
  static uint8_t lastState = 0;
  uint8_t newState = digitalRead(ENCODER_A) | (digitalRead(ENCODER_B) << 1;
  
  if(newState == lastState) return;
  
  const int8_t transitions[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  encoderCount += transitions[lastState << 2 | newState];
  lastState = newState;
}

// ======== DEBUG FUNCTIONS ========
void debugPrint(String message) {
  if(DEBUG) {
    Serial.println(message);
  }
}

// ======== EEPROM MANAGEMENT ========
#include <EEPROM.h>
struct PersistentData {
  long openPos;
  long closedPos;
  bool calibrated;
};

void savePosition() {
  PersistentData data = {
    .openPos = openPosition,
    .closedPos = closedPosition,
    .calibrated = true
  };
  EEPROM.put(0, data);
}

void loadPosition() {
  PersistentData data;
  EEPROM.get(0, data);
  if(data.calibrated) {
    openPosition = data.openPos;
    closedPosition = data.closedPos;
    encoderCount = openPosition;
  }
}

// ======== MOTOR CONTROL ========
void setPwmFrequency() {
  // WARNING: Affects PWM on pins 9 & 10
  TCCR1B = (TCCR1B & 0b11111000) | 0x05;  // 31.25Hz
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  debugPrint("[MOTOR] Stopped");
}

// ======== SAFETY FEATURES ========
void checkSoftwareEndstops() {
  if(encoderCount > openPosition + SAFETY_MARGIN || 
     encoderCount < closedPosition - SAFETY_MARGIN) {
    emergencyStop();
  }
}

void emergencyStop() {
  stopMotor();
  doorState = STOPPED;
  savePosition();
  debugPrint("[SAFETY] Emergency stop!");
}

// ======== HOMING ROUTINE ========
void homeDoor() {
  debugPrint("[HOMING] Starting...");
  doorState = HOMING;
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 100);
  
  unsigned long startTime = millis();
  while(digitalRead(HOME_SENSOR) == HIGH) {
    if(millis() - startTime > 5000) {
      debugPrint("[ERROR] Homing timeout!");
      emergencyStop();
      return;
    }
    
    // Non-blocking emergency stop check
    if(digitalRead(OPEN_BTN) == LOW) {
      debugPrint("[HOMING] User abort");
      emergencyStop();
      return;
    }
    
    delay(50);  // Collision prevention delay
  }
  
  stopMotor();
  encoderCount = openPosition;
  isHomed = true;
  doorState = IDLE;
  lastHomingTime = millis();
  debugPrint("[HOMING] Completed");
}

// ======== CALIBRATION MODE ========
void enterCalibration() {
  debugPrint("[CALIB] Starting calibration");
  
  // Open calibration
  debugPrint("[CALIB] Move to open position");
  while(digitalRead(OPEN_BTN) == HIGH) {
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 100);
    delay(10);
  }
  stopMotor();
  openPosition = encoderCount;
  
  // Close calibration
  debugPrint("[CALIB] Move to closed position");
  while(digitalRead(CLOSE_BTN) == HIGH) {
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 100);
    delay(10);
  }
  stopMotor();
  closedPosition = encoderCount;
  
  savePosition();
  debugPrint("[CALIB] Calibration done");
}

// ======== MOVEMENT CONTROL ========
bool isStuck(long* lastPos, unsigned long* lastChange) {
  if(abs(encoderCount - *lastPos) > 2) {
    *lastPos = encoderCount;
    *lastChange = millis();
    return false;
  }
  return (millis() - *lastChange) > STUCK_TIMEOUT;
}

void moveDoor(bool direction, long target) {
  digitalWrite(DIR_PIN, direction);
  const long startPos = encoderCount;
  long lastPos = encoderCount;
  unsigned long moveStart = millis();
  unsigned long lastChange = millis();

  while(millis() - moveStart < MAX_MOVE_TIME) {
    // Position-based exit
    if((direction == HIGH && encoderCount >= target) ||
       (direction == LOW && encoderCount <= target)) break;

    // Dynamic speed control
    const long remaining = abs(target - encoderCount);
    int speed = map(remaining, 0, abs(target - startPos), MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));

    // Safety checks
    if(isStuck(&lastPos, &lastChange)) {
      debugPrint("[ERROR] Motor stuck!");
      emergencyStop();
      return;
    }
    checkSoftwareEndstops();
    
    delay(10);  // Control loop timing
  }
  
  stopMotor();
}

// ======== STATE HANDLERS ========
bool checkButton(int pin) {
  static unsigned long lastPress[14] = {0};
  if(digitalRead(pin) == LOW && millis() - lastPress[pin] > DEBOUNCE_DELAY) {
    lastPress[pin] = millis();
    return true;
  }
  return false;
}

void handleIdleState() {
  // Button handling
  if(checkButton(OPEN_BTN)) {
    doorState = OPENING;
    autoCloseTimer = millis();
  }
  else if(checkButton(CLOSE_BTN)) {
    doorState = CLOSING;
  }
  
  // PIR sensor handling
  if(digitalRead(PIR_SENSOR) == LOW) {
    doorState = OPENING;
    autoCloseTimer = millis();
  }
  
  // Auto-close handling
  if(millis() - autoCloseTimer > AUTO_CLOSE_DELAY) {
    doorState = CLOSING;
  }
}

void handleMovement() {
  if(doorState == OPENING) {
    moveDoor(HIGH, openPosition);
    doorState = IDLE;
    autoCloseTimer = millis();
  }
  else if(doorState == CLOSING) {
    moveDoor(LOW, closedPosition);
    doorState = IDLE;
  }
}

void handleStoppedState() {
  if(checkButton(OPEN_BTN) || checkButton(CLOSE_BTN)) {
    debugPrint("[SYSTEM] Resuming from stop");
    doorState = IDLE;
  }
}

// ======== SETUP & MAIN LOOP ========
void setup() {
  Serial.begin(SERIAL_BAUD);
  setPwmFrequency();

  // Initialize hardware
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  // Initialize sensors
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);

  // Check for calibration mode
  if(digitalRead(OPEN_BTN) == LOW && digitalRead(CLOSE_BTN) == LOW) {
    enterCalibration();
  }

  loadPosition();
  homeDoor();
}

void loop() {
  // Periodic tasks
  static unsigned long lastMaintenanceCheck = 0;
  if(millis() - lastMaintenanceCheck > 60000) {
    lastMaintenanceCheck = millis();
    
    // Save position periodically
    if(millis() - positionSaveTimer > POS_SAVE_INTERVAL) {
      savePosition();
      positionSaveTimer = millis();
    }
    
    // Suggest homing every 24 hours
    if(millis() - lastHomingTime > HOMING_INTERVAL) {
      debugPrint("[MAINT] Periodic homing suggested");
    }
  }

  // State machine
  switch(doorState) {
    case IDLE:        handleIdleState(); break;
    case OPENING:     
    case CLOSING:     handleMovement(); break;
    case STOPPED:     handleStoppedState(); break;
    case HOMING:      break; // Handled in setup
  }
}
