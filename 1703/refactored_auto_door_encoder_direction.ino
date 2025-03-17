// === Pin Definitions ===
#define ENCODER_A 3
#define ENCODER_B 4
#define HOME_SENSOR 11
#define OPEN_BTN 5
#define CLOSE_BTN 6
#define DIR_PIN 8
#define PWM_PIN 9
#define PIR_SENSOR 12

// === Motor Speed Constants ===
#define MAX_SPEED 180
#define MIN_SPEED 40
#define SPEED_CHECK_INTERVAL 100
#define AUTO_CLOSE_DELAY 3000  // 3 seconds

// === Position Constants ===
#define MAX_POSITION 1000

// === Debounce
#define DEBOUNCE_TIME 200

// === Global State Variables ===
volatile long encoderCount = 0;
long lastEncoderCheck = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;
bool pirTriggered = false;
unsigned long lastOpenTime = 0;

unsigned long lastButtonPress = 0;
long lastEncoderPosition = 0;

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
  Serial.println("System Booted. Starting Homing...");
  homeDoor();
}

// === Main Loop ===
void loop() {
  if (!isHomed) return;

  checkButtons();
  handleAutoClose();

  delay(10); // Loop stabilization
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) > digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Homing Logic ===
void homeDoor() {
  int speed = 100;
  digitalWrite(DIR_PIN, LOW); // Closing
  analogWrite(PWM_PIN, speed);

  while (digitalRead(HOME_SENSOR) == HIGH) {
    Serial.print("Homing... Encoder: ");
    Serial.println(encoderCount);
    delay(100);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("Homing Complete.");
}

// === Handle Button Presses ===
void checkButtons() {
  if (millis() - lastButtonPress < DEBOUNCE_TIME) return;

  if (digitalRead(OPEN_BTN) == LOW) {
    lastButtonPress = millis();
    handleOpen();
  }
  else if (digitalRead(CLOSE_BTN) == LOW) {
    lastButtonPress = millis();
    handleClose();
  }
  else if (digitalRead(PIR_SENSOR) == LOW && !pirTriggered) {
    pirTriggered = true;
    handleOpen();
  }
  else if (digitalRead(PIR_SENSOR) == HIGH) {
    pirTriggered = false;
  }
}

// === Handle Door Open ===
void handleOpen() {
  if (isOpening || encoderCount <= 0) return;

  Serial.println("[ACTION] Opening Door...");
  isOpening = true;
  isClosing = false;
  moveDoor(HIGH, 0);
  lastOpenTime = millis();
}

// === Handle Door Close ===
void handleClose() {
  if (isClosing || encoderCount >= MAX_POSITION) return;

  Serial.println("[ACTION] Closing Door...");
  isClosing = true;
  isOpening = false;
  moveDoor(LOW, MAX_POSITION);
}

// === Move Door with Exponential Speed Profile ===
void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction);
  int speed;
  unsigned long startTime = millis();

  while ((direction == HIGH && encoderCount > target) ||
         (direction == LOW && encoderCount < target)) {

    long distTotal = abs(encoderCount - target);
    float brakeStartDist = 0.6 * MAX_POSITION;
    float normalized = constrain((float)distTotal / brakeStartDist, 0.0, 1.0);
    speed = (int)(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * pow(normalized, 2.5));

    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency reverse
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      emergencyStop();
      handleClose();
      return;
    }
    if (direction == LOW && (digitalRead(OPEN_BTN) == LOW || digitalRead(PIR_SENSOR) == LOW)) {
      emergencyStop();
      handleOpen();
      return;
    }

    // Encoder stuck recovery
    if (millis() - lastEncoderCheck > SPEED_CHECK_INTERVAL) {
      if (abs(encoderCount - lastEncoderPosition) < 2) {
        analogWrite(PWM_PIN, MAX_SPEED);
        Serial.println("[WARN] Encoder Stuck. Boosting Speed.");
      }
      lastEncoderCheck = millis();
      lastEncoderPosition = encoderCount;
    }

    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  isOpening = false;
  isClosing = false;
  Serial.println("[DONE] Movement Complete");
}

// === Emergency Stop ===
void emergencyStop() {
  analogWrite(PWM_PIN, 0);
  delay(200);
  Serial.println("EMERGENCY STOP");
}

// === Auto Close Logic ===
void handleAutoClose() {
  if (!isOpening && !isClosing &&
      encoderCount <= 10 &&  // Fully open
      millis() - lastOpenTime > AUTO_CLOSE_DELAY) {
    handleClose();
  }
}
