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
bool waitingForPIRToClear = false;  // ✅ New variable added
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

  homeDoor(); // Home on startup
}

// === Main Loop ===
void loop() {
  if (!isHomed) return;

  // PIR-based auto open & timer reset
  if (digitalRead(PIR_SENSOR) == LOW && !motionDetected) {
    Serial.println("[PIR] Motion detected.");
    motionDetected = true;
    lastMotionTime = millis();
    handleOpen(); // Will reset timer even if already open
  } else if (motionDetected && millis() - lastMotionTime >= PIR_RESET_DELAY && digitalRead(PIR_SENSOR) == HIGH) {
    Serial.println("[PIR] Motion ended.");
    motionDetected = false;
  }

  // Manual open/close buttons
  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen();
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }

  // Auto close logic
  handleAutoClose();
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? -1 : 1;
}

// === Homing Logic ===
void homeDoor() {
  int sp = 100;
  digitalWrite(DIR_PIN, HIGH);  // Open direction
  analogWrite(PWM_PIN, sp);
  Serial.println("[HOMING] Moving to home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    delay(10);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("[HOMING] Complete. Encoder reset to 0.");
  Serial.println("========================================");

  waitingToAutoClose = true;
  waitingForPIRToClear = true;  // ✅ Wait for PIR to clear before starting timer
  Serial.println("Waiting for PIR to clear to start auto-close timer...");
}

// === Open Logic ===
void handleOpen() {
  if (!isHomed) return;

  if (isOpening) {
    Serial.println("[INFO] Door is already opening.");
    return;
  }

  // Door is already open
  if (encoderCount >= 0 || digitalRead(HOME_SENSOR) == LOW) {
    Serial.println("[INFO] Door already open. Resetting auto-close timer.");
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;

  moveDoor(HIGH, -10);  // Move towards encoder 0
  encoderCount = 0;

  // Reset auto-close states
  waitingToAutoClose = true;
  waitingForPIRToClear = true;  // ✅ Always wait for PIR to go HIGH after open
  motionDetected = false;
  Serial.println("Waiting for PIR to clear to start auto-close timer...");
}

// === Close Logic ===
void handleClose() {
  if (!isHomed) return;

  if(isOpening){
    Serial.println("[CLOSE] Ignored - Door is opening");
    return;
  }

  if (isClosing) {
    Serial.println("[INFO] Door is already closing.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Door already closed.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;

  moveDoor(LOW, MAX_POSITION); // Move toward MAX_POSITION
}

// === Move Door ===
void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction);
  int speed = MAX_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && digitalRead(HOME_SENSOR) == HIGH) || (direction == LOW && encoderCount > target)) {
    if (direction == HIGH && digitalRead(HOME_SENSOR) == LOW) break;

    int distanceToTarget = abs(encoderCount - target);
    speed = map(distanceToTarget, 0, abs(MAX_POSITION), MIN_SPEED, MAX_SPEED);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    if ((direction == LOW && digitalRead(OPEN_BTN) == LOW) || digitalRead(PIR_SENSOR) == LOW) {
      Serial.println("[EMERGENCY] Close interrupted by Open.");
      emergencyStop();
      handleOpen();
      return;
    }

    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[INFO] Close ignored during opening.");
    }

    delay(10);
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

// === Auto-Close Handler ===
void handleAutoClose() {
  if (!waitingToAutoClose || isOpening || isClosing) return;

  if (waitingForPIRToClear) {
    if (digitalRead(PIR_SENSOR) == HIGH) {
      Serial.println("[AUTO] PIR clear. Starting auto-close timer...");
      autoCloseStartTime = millis();
      waitingForPIRToClear = false;
    } else {
      return; // Still motion — wait
    }
  }

  if (motionDetected) {
    waitingForPIRToClear = true;
    return;
  }

  if (millis() - autoCloseStartTime >= AUTO_CLOSE_DELAY) {
    Serial.println("[AUTO] Auto-closing triggered.");
    handleClose();
    waitingToAutoClose = false;
  }
}
