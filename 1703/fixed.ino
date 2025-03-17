// === PIN DEFINITIONS ===
#define ENCODER_A     3
#define ENCODER_B     4
#define MOTOR_PWM     9
#define MOTOR_DIR     8
#define OPEN_BTN      5
#define CLOSE_BTN     6
#define HOME_SENSOR  11
#define PIR_SENSOR    12

// === ENCODER & DOOR STATE ===
volatile long encoderCount = 0;
volatile bool isOpening = false;
volatile bool isClosing = false;
bool isHomed = false;
bool pirTriggered = false;

// === TIMERS ===
unsigned long lastOpenTime = 0;
unsigned long pirCooldownTime = 0;
unsigned long lastOpenBtnPress = 0;
unsigned long lastCloseBtnPress = 0;

// === CONFIG ===
const long MAX_POSITION       = 1000;
#define OPEN_SLOWDOWN         ((int)(0.4 * MAX_POSITION))
#define CLOSE_SLOWDOWN        ((int)(0.6 * MAX_POSITION))

const int MOTOR_SPEED         = 150;
const int MOTOR_SLOW_SPEED    = 80;
const int SPEED_RAMP          = 2;

const unsigned long AUTO_CLOSE_DELAY = 5000;
const unsigned long PIR_COOLDOWN     = 5000;
const unsigned long MOVE_TIMEOUT     = 10000;
const unsigned long DEBOUNCE_DELAY   = 200;

// === INTERRUPT ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === MOTOR CONTROL ===
void setMotor(int dir, int speed) {
  digitalWrite(MOTOR_DIR, dir);
  analogWrite(MOTOR_PWM, speed);
}

void stopMotor() {
  analogWrite(MOTOR_PWM, 0);
  isOpening = false;
  isClosing = false;
  Serial.println("[MOTOR] Stopped");
}

// === MOVEMENT LOGIC ===
void moveDoor(int direction, long targetDelta) {
  long startCount = encoderCount;
  long target = direction == HIGH ? startCount + targetDelta : startCount - targetDelta;
  int speed = 0;

  setMotor(direction, speed);
  unsigned long startTime = millis();

  while ((direction == HIGH && encoderCount < target) || (direction == LOW && encoderCount > target)) {
    if (millis() - startTime > MOVE_TIMEOUT) {
      Serial.println("[ERROR] Door movement timeout.");
      break;
    }

    // Emergency interrupts
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[INTERRUPT] Opening stopped by Close button.");
      stopMotor();
      isOpening = false;
      isClosing = true;
      return;
    }
    if (direction == LOW && (digitalRead(OPEN_BTN) == LOW || digitalRead(PIR_SENSOR) == LOW)) {
      Serial.println("[INTERRUPT] Closing stopped by Open or Motion.");
      stopMotor();
      isClosing = false;
      isOpening = true;
      return;
    }

    long distanceLeft = abs(encoderCount - target);
    bool inSlowZone = distanceLeft < ((direction == HIGH) ? OPEN_SLOWDOWN : CLOSE_SLOWDOWN);

    speed = inSlowZone ? max(speed - SPEED_RAMP, MOTOR_SLOW_SPEED)
                       : min(speed + SPEED_RAMP, MOTOR_SPEED);

    setMotor(direction, speed);
    delay(10);
  }

  stopMotor();
}

// === OPERATIONS ===
void handleOpen() {
  if (!isHomed || isOpening || encoderCount >= MAX_POSITION) return;
  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  moveDoor(HIGH, MAX_POSITION - encoderCount);
  isOpening = false;
  lastOpenTime = millis();
}

void handleClose() {
  if (!isHomed || isClosing || encoderCount <= 0) return;
  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  moveDoor(LOW, encoderCount);
  isClosing = false;
}

void homeDoor() {
  Serial.println("[HOMING] Searching home...");
  setMotor(HIGH, MOTOR_SLOW_SPEED);
  while (digitalRead(HOME_SENSOR) == HIGH) delay(10);
  stopMotor();
  encoderCount = 0;
  isHomed = true;
  Serial.println("[HOMING] Done. Encoder reset.");
}

// === INPUT HANDLING ===
bool isButtonPressed(uint8_t pin, unsigned long &lastPressTime) {
  if (digitalRead(pin) == LOW && millis() - lastPressTime > DEBOUNCE_DELAY) {
    lastPressTime = millis();
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(9600);

  // === Pin Modes ===
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);

  // === Attach Encoder ISR ===
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);

  // === Home on Startup ===
  homeDoor();
}

void loop() {
  // === Button Inputs with Debounce ===
  if (isButtonPressed(OPEN_BTN, lastOpenBtnPress)) handleOpen();
  else if (isButtonPressed(CLOSE_BTN, lastCloseBtnPress)) handleClose();

  // === PIR Motion Detection ===
  if (digitalRead(PIR_SENSOR) == LOW && !pirTriggered && millis() - pirCooldownTime > PIR_COOLDOWN) {
    Serial.println("[PIR] Motion detected. Triggering open.");
    handleOpen();
    pirTriggered = true;
    pirCooldownTime = millis();
  } else if (digitalRead(PIR_SENSOR) == HIGH && pirTriggered) {
    pirTriggered = false;
  }

  // === Auto Close Logic ===
  if (!isClosing && encoderCount >= MAX_POSITION && millis() - lastOpenTime > AUTO_CLOSE_DELAY) {
    Serial.println("[AUTO] Auto-closing due to timeout...");
    handleClose();
  }

  // === Serial Debug Output ===
  Serial.print("ENC: ");
  Serial.println(encoderCount);
  delay(100);
}
