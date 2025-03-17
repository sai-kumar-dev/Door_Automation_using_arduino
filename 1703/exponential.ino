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
#define DEBOUNCE_DELAY 100

// === Door Position Constants ===
#define MAX_POSITION -1000
#define OPEN_SLOWDOWN 0.4 * MAX_POSITION
#define CLOSE_SLOWDOWN 0.6 * MAX_POSITION

// === State Variables ===
volatile int encoderCount = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;
bool pirTriggered = false;

unsigned long lastBtnTime = 0;

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
  Serial.println("System Booted - Starting Homing...");
  homeDoor();
}

// === Main Loop ===
void loop() {
  Serial.print("Encoder: ");
  Serial.println(encoderCount);

  if (!isHomed) return;

  // Debounce logic
  if (millis() - lastBtnTime < DEBOUNCE_DELAY) return;

  if (digitalRead(PIR_SENSOR) == LOW && !pirTriggered) {
    pirTriggered = true;
    lastBtnTime = millis();
    Serial.println("[PIR] Motion detected. Opening door.");
    handleOpen();
  } else if (digitalRead(PIR_SENSOR) == HIGH) {
    pirTriggered = false;
  }

  if (digitalRead(OPEN_BTN) == LOW) {
    lastBtnTime = millis();
    handleOpen();
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    lastBtnTime = millis();
    handleClose();
  }
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Homing Function ===
void homeDoor() {
  int speed = 100;
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, speed);

  while (digitalRead(HOME_SENSOR) == HIGH) {
    Serial.print("Homing - Encoder: ");
    Serial.println(encoderCount);

    // Smoothly reduce speed using exponential decay
    speed = max(MIN_SPEED, speed * 0.95);
    analogWrite(PWM_PIN, speed);
    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("Homing Complete. Encoder set to 0.");
}

// === Open Handler ===
void handleOpen() {
  if (isOpening) {
    Serial.println("[INFO] Already opening.");
    return;
  }
  if (encoderCount >= 0) {
    Serial.println("[INFO] Already fully open.");
    return;
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;

  moveDoor(HIGH, OPEN_SLOWDOWN, 0);
  encoderCount = 0;
  isOpening = false;
}

// === Close Handler ===
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
  isClosing = false;
}

// === Motor Movement with Smooth Speed Profile ===
void moveDoor(bool direction, int slowdownPoint, int targetPosition) {
  digitalWrite(DIR_PIN, direction);
  int speed = MAX_SPEED;
  analogWrite(PWM_PIN, speed);

  Serial.println(direction == HIGH ? "[OPENING]" : "[CLOSING]");

  while ((direction == HIGH && encoderCount < targetPosition) || 
         (direction == LOW && encoderCount > targetPosition)) {

    // Stop if homing sensor is triggered during opening
    if (direction == HIGH && digitalRead(HOME_SENSOR) == LOW) {
      break;
    }

    // Calculate distance to slowdown point
    int distanceToSlow = abs(encoderCount - slowdownPoint);
    if (distanceToSlow < 600) {
      // Exponential slowdown
      float curve = exp(-0.005 * distanceToSlow);
      speed = MAX_SPEED * curve;
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    }

    analogWrite(PWM_PIN, speed);

    Serial.print("[ENC] Pos: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(speed);

    // Emergency stop logic
    if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
      Serial.println("[EMERGENCY] Opening interrupted by close.");
      emergencyStop();
      handleClose();
      return;
    }
    if (direction == LOW && (digitalRead(OPEN_BTN) == LOW || digitalRead(PIR_SENSOR) == LOW)) {
      Serial.println("[EMERGENCY] Closing interrupted by open.");
      emergencyStop();
      handleOpen();
      return;
    }

    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  Serial.println("[DONE] Movement complete.");
}

// === Emergency Stop ===
void emergencyStop() {
  analogWrite(PWM_PIN, 0);
  delay(200);
  Serial.println("[INFO] Motor stopped.");
}
