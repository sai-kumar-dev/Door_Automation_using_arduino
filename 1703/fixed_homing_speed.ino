// === Pin Definitions ===
#define ENCODER_A     3
#define ENCODER_B     4
#define HOME_SENSOR   11
#define OPEN_BTN      5
#define CLOSE_BTN     6
#define DIR_PIN       8
#define PWM_PIN       9
#define PIR_SENSOR    12

// === Speed Constants ===
#define MAX_SPEED     180
#define MIN_SPEED     40
#define HOMING_MIN    50
#define HOMING_MAX    120
#define SPEED_STEP    10
#define ACCEL_STEP    2
#define DECEL_ZONE    200
#define MOVE_TIMEOUT  10000  // 10 seconds timeout during homing

// === Position Constants ===
#define MAX_POSITION -1000
#define OPEN_SLOWDOWN  (0.4 * MAX_POSITION)
#define CLOSE_SLOWDOWN (0.6 * MAX_POSITION)

// === State Flags ===
volatile int encoderCount = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;
bool pirTriggered = false;

// === Debounce Timing ===
unsigned long lastOpenPress = 0;
unsigned long lastClosePress = 0;
const unsigned long debounceDelay = 200;

// === Setup ===
void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PIR_SENSOR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);

  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Door System Booted ===");
  homeDoor();
}

// === Loop ===
void loop() {
  if (!isHomed) return;

  Serial.print("Encoder: ");
  Serial.println(encoderCount);

  if (digitalRead(PIR_SENSOR) == LOW && !pirTriggered) {
    Serial.println("[PIR] Motion detected - Opening door.");
    pirTriggered = true;
    handleOpen();
  } else if (digitalRead(PIR_SENSOR) == HIGH && pirTriggered) {
    pirTriggered = false;
  }

  if (digitalRead(OPEN_BTN) == LOW && millis() - lastOpenPress > debounceDelay) {
    lastOpenPress = millis();
    handleOpen();
  }

  if (digitalRead(CLOSE_BTN) == LOW && millis() - lastClosePress > debounceDelay) {
    lastClosePress = millis();
    handleClose();
  }

  delay(50);
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Motor Control Helpers ===
void setMotor(bool dir, int pwmVal) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, pwmVal);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  Serial.println("[INFO] Motor stopped.");
}

// === Smooth Homing Logic ===
void homeDoor() {
  Serial.println("[HOMING] Starting...");
  long startPos = encoderCount;
  int speed = HOMING_MIN;
  unsigned long startTime = millis();
  bool slowing = false;

  setMotor(HIGH, speed);

  while (digitalRead(HOME_SENSOR) == HIGH) {
    long progress = abs(encoderCount - startPos);

    if (progress > DECEL_ZONE) slowing = true;

    // Smooth acceleration/deceleration
    if (!slowing && speed < HOMING_MAX) speed += ACCEL_STEP;
    else if (slowing && speed > HOMING_MIN) speed -= ACCEL_STEP;

    speed = constrain(speed, HOMING_MIN, HOMING_MAX);
    analogWrite(PWM_PIN, speed);

    Serial.print("[HOMING] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    if (millis() - startTime > MOVE_TIMEOUT) {
      Serial.println("[ERROR] Homing timed out.");
      stopMotor();
      return;
    }

    delay(10);
  }

  stopMotor();
  encoderCount = 0;
  isHomed = true;
  Serial.println("[HOMING] Completed. Encoder reset to 0.");
  Serial.println("========================================");
}

// === Door Movement Functions ===
void handleOpen() {
  if (isOpening || pirTriggered) {
    Serial.println("[INFO] Door already opening.");
    return;
  }

  if (encoderCount >= 0) {
    Serial.println("[INFO] Door is fully open.");
    return;
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;
  moveDoor(HIGH, OPEN_SLOWDOWN, -20);
  encoderCount = 0;
}

void handleClose() {
  if (isClosing) {
    Serial.println("[INFO] Door already closing.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Door is fully closed.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;
  moveDoor(LOW, CLOSE_SLOWDOWN, MAX_POSITION);
}

// === Move Door with S-Curve-Like Slowdown ===
void moveDoor(bool dir, int slowdownPoint, int target) {
  setMotor(dir, MAX_SPEED);
  int speed = MAX_SPEED;

  Serial.print("[MOVING] ");
  Serial.println(dir == HIGH ? "Opening..." : "Closing...");

  while ((dir == HIGH && encoderCount <= target) || (dir == LOW && encoderCount >= target)) {
    int distToSlow = abs(encoderCount - slowdownPoint);

    if (distToSlow < 600) {
      speed = map(distToSlow, 600, 0, MAX_SPEED, MIN_SPEED);
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    }

    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency Stops
    if (dir == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[EMERGENCY] Open interrupted by Close.");
      emergencyStop();
      handleClose();
      return;
    }

    if (dir == LOW && (digitalRead(OPEN_BTN) == LOW || digitalRead(PIR_SENSOR) == LOW)) {
      Serial.println("[EMERGENCY] Close interrupted by Open.");
      emergencyStop();
      handleOpen();
      return;
    }

    if (dir == HIGH && digitalRead(HOME_SENSOR) == LOW) break;

    delay(50);
  }

  stopMotor();
  isOpening = false;
  isClosing = false;
  Serial.println("[DONE] Door movement complete.");
  Serial.println("========================================");
}

void emergencyStop() {
  stopMotor();
  delay(200);
}
