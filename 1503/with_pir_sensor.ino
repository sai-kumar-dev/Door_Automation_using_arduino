// === Pin Definitions ===
#define ENCODER_A 3            // Encoder channel A (interrupt pin)
#define ENCODER_B 4            // Encoder channel B
#define HOME_SENSOR 11         // Home position sensor (active LOW with INPUT_PULLUP)
#define OPEN_BTN 5             // Open button (active LOW)
#define CLOSE_BTN 6            // Close button (active LOW)
#define DIR_PIN 8              // Motor direction pin
#define PWM_PIN 9              // Motor PWM speed control
#define PIR_SENSOR 7           // PIR motion detection sensor (HIGH when person is detected)

// === Motor Speed Control Constants ===
#define MAX_SPEED 200          // Highest motor speed (PWM)
#define MIN_SPEED 40           // Minimum speed near end travel

// === Door Positioning Constants ===
#define MAX_POSITION -1000     // Encoder count when door is fully closed
#define OPEN_SLOWDOWN 0.2 * MAX_POSITION   // Encoder position to begin slowdown during opening
#define CLOSE_SLOWDOWN 0.8 * MAX_POSITION  // Encoder position to begin slowdown during closing

// === Auto-Close Constants ===
#define AUTO_CLOSE_DELAY 5000          // Delay before auto-close in milliseconds
#define EXTENDED_CLOSE_DELAY 8000      // Delay if PIR or open pressed during opening

// === State Variables ===
volatile int encoderCount = 0;         // Tracks door position
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;

unsigned long lastOpenTime = 0;        // For auto-close timing
bool autoCloseScheduled = false;

// === Setup Function ===
void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PIR_SENSOR, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);

  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Door System Booted ===");
  Serial.println("Starting homing sequence...");
  homeDoor();
}

void loop() {
  if (!isHomed) return;

  // Check open/close button
  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen(true); // true = manual trigger
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }

  // PIR sensor logic
  if (digitalRead(PIR_SENSOR) == HIGH && encoderCount < 0 && !isOpening) {
    Serial.println("[PIR] Human detected. Triggering open.");
    handleOpen(false); // false = automatic trigger
  }

  // Auto-close logic
  if (autoCloseScheduled && millis() - lastOpenTime >= AUTO_CLOSE_DELAY) {
    if (encoderCount > MAX_POSITION) {
      Serial.println("[AUTO] Auto-closing door after delay.");
      handleClose();
      autoCloseScheduled = false;
    }
  }

  delay(100); // Basic debounce and loop rate
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Homing Logic ===
void homeDoor() {
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, MAX_SPEED);
  Serial.println("[HOMING] Moving to home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    Serial.print("[HOMING] Encoder: ");
    Serial.println(encoderCount);
    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("[HOMING] Complete. Encoder = 0.");
  Serial.println("========================================");
}

// === Handle Open Button or PIR Trigger ===
void handleOpen(bool isManual) {
  if (isOpening) {
    if (isManual) Serial.println("[INFO] Already opening.");
    lastOpenTime = millis(); // Extend close delay
    return;
  }

  if (encoderCount >= 0) {
    Serial.println("[INFO] Door already open.");
    return;
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;

  // Start movement to home (0)
  moveDoor(HIGH, OPEN_SLOWDOWN, 0);

  // Schedule auto-close
  lastOpenTime = millis();
  AUTO_CLOSE_DELAY = isManual ? EXTENDED_CLOSE_DELAY : AUTO_CLOSE_DELAY;
  autoCloseScheduled = true;
}

// === Handle Close Button ===
void handleClose() {
  if (isClosing) {
    Serial.println("[INFO] Already closing.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Already fully closed.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;

  moveDoor(LOW, CLOSE_SLOWDOWN, MAX_POSITION);
}

// === Smooth Motor Movement ===
void moveDoor(bool direction, int slowdownPoint, int target) {
  digitalWrite(DIR_PIN, direction);
  int speed = MAX_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && encoderCount < target) ||
         (direction == LOW && encoderCount > target)) {

    int distanceToSlowdown = abs(encoderCount - slowdownPoint);
    if (distanceToSlowdown < 600) {
      speed = map(distanceToSlowdown, 600, 0, MAX_SPEED, MIN_SPEED);
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    }

    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] ");
    Serial.print("Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency Interruption Handling
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[EMERGENCY] Open interrupted by Close.");
      emergencyStop();
      handleClose();
      return;
    }

    if (direction == LOW && digitalRead(OPEN_BTN) == LOW) {
      Serial.println("[EMERGENCY] Close interrupted by Open.");
      emergencyStop();
      handleOpen(true);
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
