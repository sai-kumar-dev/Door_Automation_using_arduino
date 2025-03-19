
// === Motor & Encoder Pins ===
#define DIR_PIN     8
#define PWM_PIN     9
#define ENCODER_A   3
#define ENCODER_B   4

// === Sensor & Button Pins ===
#define HOME_SENSOR 11
#define PIR_SENSOR  12
#define OPEN_BTN    5
#define CLOSE_BTN   6

// === Motor Speeds ===
#define MAX_SPEED   180
#define MIN_SPEED   40

// === Encoder Positions ===
#define CLOSED_POSITION -1050  // fully closed position
#define OPEN_POSITION    0     // fully open

// === State Flags ===
bool isOpening = false;
bool isClosing = false;
bool isHomed = false;
bool motionDetected = false;

// === Encoder Count ===
volatile long encoderCount = 0;

// === Auto-Close Timer ===
unsigned long autoCloseStartTime = 0;
const unsigned long AUTO_CLOSE_DELAY = 5000;

// === Debounce Timers ===
unsigned long lastButtonPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 250;

// === States ===
enum DoorState { IDLE, OPENING, CLOSING, HOMING, STOPPED };
DoorState currentState = HOMING;

// === Function Declarations ===
void moveDoor(bool direction, long targetPosition);
void stopMotor();
void resetAutoCloseTimer();
bool debounceButton(int pin);
void homeDoor();
void handleOpen();
void handleClose();

// === Encoder ISR ===
void updateEncoder() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  encoderCount += (a > b) ? -1 : 1;
}

void setup() {
  Serial.begin(9600);

  // Motor
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  // Sensors & Buttons
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);

  // Start with homing
  homeDoor();
}

void loop() {
  if (!isHomed) return;

  switch (currentState) {
    case IDLE:
      if (digitalRead(PIR_SENSOR) == LOW) {
        Serial.println("[PIR] Motion detected");
        handleOpen();
        resetAutoCloseTimer();
      }

      if (millis() - autoCloseStartTime >= AUTO_CLOSE_DELAY) {
        handleClose();
      }

      if (debounceButton(OPEN_BTN)) {
        handleOpen();
        resetAutoCloseTimer();
      }

      if (debounceButton(CLOSE_BTN)) {
        handleClose();
      }
      break;

    case CLOSING:
      if (debounceButton(OPEN_BTN)) {
        Serial.println("[EMERGENCY] Open pressed while closing");
        stopMotor();
        handleOpen();
      }
      break;

    case STOPPED:
      if (debounceButton(OPEN_BTN)) {
        handleOpen();
        resetAutoCloseTimer();
      }
      if (debounceButton(CLOSE_BTN)) {
        handleClose();
      }
      break;

    case HOMING:
      // Do nothing
      break;
  }
}

void homeDoor() {
  Serial.println("[INIT] Homing...");

  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 100);

  unsigned long start = millis();
  while (digitalRead(HOME_SENSOR) == HIGH) {
    if (millis() - start > 5000) {
      Serial.println("[ERROR] Homing timeout!");
      stopMotor();
      return;
    }
    delay(50);
  }

  stopMotor();
  encoderCount = 0;
  isHomed = true;
  currentState = IDLE;
  Serial.println("[INFO] Homing complete");
}

void handleOpen() {
  if (isOpening || encoderCount >= OPEN_POSITION) return;
  currentState = OPENING;
  moveDoor(HIGH, OPEN_POSITION);
  stopMotor();
  encoderCount = 0; // Reset encoder after full open
  currentState = IDLE;
}

void handleClose() {
  if (isClosing || encoderCount <= CLOSED_POSITION) return;
  currentState = CLOSING;
  moveDoor(LOW, CLOSED_POSITION);
  stopMotor();
  currentState = IDLE;
}

void moveDoor(bool direction, long target) {
  digitalWrite(DIR_PIN, direction);
  long distance = abs(target - encoderCount);

  while (true) {
    long currentDist = abs(target - encoderCount);
    if ((direction == HIGH && digitalRead(HOME_SENSOR) == LOW) ||
        (direction == LOW && encoderCount <= target)) {
      break;
    }

    int speed = map(currentDist, 0, distance, MIN_SPEED, MAX_SPEED);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, speed);

    delay(10);
  }
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  isOpening = false;
  isClosing = false;
  Serial.println("[MOTOR] Stopped");
}

void resetAutoCloseTimer() {
  autoCloseStartTime = millis();
}

bool debounceButton(int pin) {
  if (digitalRead(pin) == LOW && (millis() - lastButtonPressTime > DEBOUNCE_DELAY)) {
    lastButtonPressTime = millis();
    return true;
  }
  return false;
}
