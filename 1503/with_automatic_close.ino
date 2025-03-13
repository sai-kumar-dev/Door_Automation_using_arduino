// === Pin Definitions ===
#define ENCODER_A 3
#define ENCODER_B 4
#define HOME_SENSOR 11
#define OPEN_BTN 5
#define CLOSE_BTN 6
#define DIR_PIN 8
#define PWM_PIN 9

// === Motor Speed Control Constants ===
#define MAX_SPEED 200
#define MIN_SPEED 40
#define SPEED_STEP 10

// === Door Positioning Constants ===
#define MAX_POSITION -1000
#define OPEN_SLOWDOWN 0.2 * MAX_POSITION
#define CLOSE_SLOWDOWN 0.8 * MAX_POSITION

// === Auto-Close Timing ===
#define AUTO_CLOSE_DELAY 5000  // Default 5 seconds
unsigned long autoCloseStartTime = 0;
bool autoClosePending = false;

// === State Variables ===
volatile int encoderCount = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;

// === Setup Function ===
void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);

  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Door System Booted ===");
  Serial.println("Starting homing sequence...");
  homeDoor();
}

// === Main Loop ===
void loop() {
  if (!isHomed) return;

  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen();
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }

  // Check for auto-close condition
  if (autoClosePending && millis() - autoCloseStartTime >= AUTO_CLOSE_DELAY) {
    Serial.println("[AUTO] Delay elapsed. Closing door automatically.");
    autoClosePending = false;
    handleClose();
  }

  delay(100);
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Homing Logic ===
void homeDoor() {
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, MAX_SPEED);
  Serial.println("[Homing] Moving towards home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    Serial.print("[Homing] Encoder: ");
    Serial.println(encoderCount);
    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("[Homing] Completed. Encoder reset to 0.");
  Serial.println("========================================");
  Serial.println("Ready for operation. Awaiting input...");

  // Start auto-close countdown after homing
  startAutoCloseTimer();
}

// === Handle Open Button Press ===
void handleOpen() {
  if (isOpening) {
    Serial.println("[INFO] Door is already opening. Extending auto-close timer.");
    startAutoCloseTimer();  // Extend timer on repeated open
    return;
  }

  if (encoderCount >= 0) {
    Serial.println("[INFO] Door is already fully open.");
    return;
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;
  moveDoor(HIGH, OPEN_SLOWDOWN, 0);

  // Start auto-close after reaching open
  startAutoCloseTimer();
}

// === Handle Close Button Press ===
void handleClose() {
  if (isClosing) {
    Serial.println("[INFO] Door is already closing.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Door is already fully closed.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;
  moveDoor(LOW, CLOSE_SLOWDOWN, MAX_POSITION);
}

// === General Motor Movement ===
void moveDoor(bool direction, int slowdownPoint, int target) {
  digitalWrite(DIR_PIN, direction);
  int speed = MAX_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && encoderCount < target) || (direction == LOW && encoderCount > target)) {
    int distanceToSlowdown = abs(encoderCount - slowdownPoint);

    if (distanceToSlowdown < 600) {
      speed = map(distanceToSlowdown, 600, 0, MAX_SPEED, MIN_SPEED);
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    }

    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency Interrupts
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[EMERGENCY] Open interrupted by Close. Stopping...");
      emergencyStop();
      handleClose();
      return;
    }

    if (direction == LOW && digitalRead(OPEN_BTN) == LOW) {
      Serial.println("[EMERGENCY] Close interrupted by Open. Stopping...");
      emergencyStop();
      handleOpen();
      return;
    }

    delay(50);
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
  delay(200);
  Serial.println("[INFO] Motor stopped.");
}

// === Auto-Close Timer Logic ===
void startAutoCloseTimer() {
  autoCloseStartTime = millis();
  autoClosePending = true;
  Serial.print("[TIMER] Auto-close will trigger in ");
  Serial.print(AUTO_CLOSE_DELAY / 1000);
  Serial.println(" seconds.");
}
