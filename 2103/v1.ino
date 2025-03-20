// === Pin Definitions ===
#define ENCODER_A 3
#define ENCODER_B 4
#define HOME_SENSOR 11
#define OPEN_BTN 5
#define CLOSE_BTN 6
#define DIR_PIN 8
#define PWM_PIN 9
#define PIR_SENSOR 12

// === Motor Speed Control Constants ===
#define MAX_SPEED 180
#define MIN_SPEED 40

// === Door Positioning Constants ===
#define MAX_POSITION -1060

// === Auto-Close Timing Constants ===
#define AUTO_CLOSE_DELAY 5000
#define PIR_RESET_DELAY 2000

// === State Variables ===
volatile int encoderCount = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;

bool waitingToAutoClose = false;
unsigned long autoCloseStartTime = 0;
unsigned long lastMotionTime = 0;

bool motionDetected = false;

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
  delay(2000);
  Serial.println("=== System Booted ===");
  homeDoor();
}

// === Main Loop ===
void loop() {
  if (!isHomed) return;

  // PIR Handling
  if (digitalRead(PIR_SENSOR) == LOW && !motionDetected) {
    Serial.println("[PIR] Motion detected.");
    motionDetected = true;
    lastMotionTime = millis();
    handleOpen();
  } else if (motionDetected && millis() - lastMotionTime >= PIR_RESET_DELAY && digitalRead(PIR_SENSOR) == HIGH) {
    Serial.println("[PIR] Motion ended.");
    motionDetected = false;
  }

  // Button Handling
  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen();
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }

  handleAutoClose();
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? -1 : 1;
}

// === Homing Logic ===
void homeDoor() {
  digitalWrite(DIR_PIN, HIGH);  // Open direction
  analogWrite(PWM_PIN, 100);
  Serial.println("[HOMING] Moving to home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    delay(10);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;

  Serial.println("[HOMING] Complete. Encoder reset to 0.");
  Serial.println("========================================");

  // Start auto-close countdown immediately after homing (door is now fully open)
  waitingToAutoClose = true;
  autoCloseStartTime = millis();
  Serial.println("[AUTO] Started auto-close countdown after homing.");
}

// === Open Logic ===
void handleOpen() {
  if (!isHomed) return;

  if (isOpening) {
    Serial.println("[INFO] Door is already opening.");
    return;
  }

  if (encoderCount >= 0 || digitalRead(HOME_SENSOR) == LOW) {
    // Door is already open
    encoderCount = 0;
    Serial.println("[INFO] Door is already open. Resetting auto-close timer.");
    waitingToAutoClose = true;
    autoCloseStartTime = millis();
    return;
  }

  // If door is closing, interrupt it and reopen
  if (isClosing) {
    Serial.println("[INTERRUPT] Opening requested during closing. Reversing...");
    emergencyStop();
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;

  moveDoor(HIGH, 0);  // Move toward home sensor (encoder 0)
  encoderCount = 0;

  waitingToAutoClose = true;
  autoCloseStartTime = millis();
  motionDetected = false;
  Serial.println("[AUTO] Auto-close timer reset after open.");
}

// === Close Logic ===
void handleClose() {
  if (!isHomed) return;

  if (isClosing) {
    Serial.println("[INFO] Door is already closing.");
    return;
  }

  if (isOpening) {
    // Ignore close command during opening
    Serial.println("[INFO] Ignored close request during opening.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Door is already closed.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;

  moveDoor(LOW, MAX_POSITION);
}

// === Move Door ===
void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction);
  int speed = MAX_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && digitalRead(HOME_SENSOR) == HIGH) ||
         (direction == LOW && encoderCount > target)) {

    // Break if home sensor is triggered during open
    if (direction == HIGH && digitalRead(HOME_SENSOR) == LOW) break;

    int distanceToTarget = abs(encoderCount - target);
    speed = map(distanceToTarget, 0, abs(MAX_POSITION), MIN_SPEED, MAX_SPEED);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Interrupt during close: Open button or PIR detected
    if (direction == LOW && (digitalRead(OPEN_BTN) == LOW || digitalRead(PIR_SENSOR) == LOW)) {
      Serial.println("[INTERRUPT] Open triggered during closing.");
      emergencyStop();
      handleOpen();
      return;
    }

    delay(10);  // avoid watchdog timer trip
  }

  analogWrite(PWM_PIN, 0);
  Serial.println("[DONE] Door movement complete.");
  isOpening = false;
  isClosing = false;
  Serial.println("========================================");
}

// === Emergency Stop ===
void emergencyStop() {
  analogWrite(PWM_PIN, 0);
  delay(100);
  Serial.println("[EMERGENCY] Motor stopped.");
}

// === Auto-Close Logic ===
void handleAutoClose() {
  if (!waitingToAutoClose || isOpening || isClosing) return;

  if (motionDetected) {
    // Reset timer if motion still going
    autoCloseStartTime = millis();
    return;
  }

  if (millis() - autoCloseStartTime >= AUTO_CLOSE_DELAY) {
    Serial.println("[AUTO] Auto-close timer expired. Closing door...");
    waitingToAutoClose = false;
    handleClose();
  }
}
