// === Pin Definitions ===
#define ENCODER_A 3
#define ENCODER_B 4
#define HOME_SENSOR 11
#define OPEN_BTN 5
#define CLOSE_BTN 6
#define DIR_PIN 8
#define PWM_PIN 9
#define PIR_PIN 7

// === Constants ===
#define MAX_POSITION -1000
#define AUTO_CLOSE_DELAY 5000 // ms
#define KP 0.5
#define KD 0.1
#define KI 0.01
#define MAX_PWM 200
#define MIN_PWM 40

// === Global Variables ===
volatile int encoderCount = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;
bool motionDetected = false;

unsigned long lastOpenedTime = 0;
bool autoCloseScheduled = false;

// PID variables
float ePrevious = 0;
float eIntegral = 0;
unsigned long previousTime = 0;

// === Setup ===
void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);
  Serial.begin(115200);
  delay(2000);

  Serial.println("System Booting...");
  homeDoor();
}

// === Loop ===
void loop() {
  if (!isHomed) return;

  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen(true);
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }

  // PIR Sensor check
  if (digitalRead(PIR_PIN) == HIGH) {
    Serial.println("[PIR] Motion detected near door.");
    motionDetected = true;
    handleOpen(false); // motion-triggered open
  }

  // Auto-close logic
  if (autoCloseScheduled && millis() - lastOpenedTime > AUTO_CLOSE_DELAY) {
    Serial.println("[AUTO] Closing after delay...");
    autoCloseScheduled = false;
    handleClose();
  }

  delay(100); // debounce
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

// === Homing ===
void homeDoor() {
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, MAX_PWM);
  Serial.println("[Homing] Moving to home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    delay(50);
  }

  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("[Homing] Complete. Encoder set to 0.");
}

// === PID Controller ===
float pidController(int target) {
  unsigned long currentTime = micros();
  float deltaT = (currentTime - previousTime) / 1.0e6;

  int error = target - encoderCount;
  float eDerivative = (error - ePrevious) / deltaT;
  eIntegral += error * deltaT;

  float output = KP * error + KD * eDerivative + KI * eIntegral;

  ePrevious = error;
  previousTime = currentTime;

  return constrain(abs(output), MIN_PWM, MAX_PWM);
}

// === Open Button Handling ===
void handleOpen(bool isButtonPress) {
  if (isOpening || encoderCount >= 0) {
    Serial.println("[INFO] Door already open or opening.");
    return;
  }

  Serial.println("[ACTION] Opening door...");
  isOpening = true;
  isClosing = false;

  moveDoor(HIGH, 0);

  lastOpenedTime = millis();
  if (isButtonPress) {
    AUTO_CLOSE_DELAY += 2000;  // extend auto-close if opened manually
    Serial.println("[INFO] Delay extended due to button press.");
  }
  autoCloseScheduled = true;
}

// === Close Button Handling ===
void handleClose() {
  if (isClosing || encoderCount <= MAX_POSITION) {
    Serial.println("[INFO] Door already closed or closing.");
    return;
  }

  Serial.println("[ACTION] Closing door...");
  isClosing = true;
  isOpening = false;

  moveDoor(LOW, MAX_POSITION);
  motionDetected = false;
}

// === Move Door with PID ===
void moveDoor(bool direction, int target) {
  digitalWrite(DIR_PIN, direction);
  Serial.print("[MOVING] ");
  Serial.println(direction == HIGH ? "Opening..." : "Closing...");

  while ((direction == HIGH && encoderCount < target) || (direction == LOW && encoderCount > target)) {
    float pwm = pidController(target);
    analogWrite(PWM_PIN, pwm);

    Serial.print("[ENC] Pos: ");
    Serial.print(encoderCount);
    Serial.print(" | PWM: ");
    Serial.println(pwm);

    // Emergency Interrupts
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
  isOpening = false;
  isClosing = false;
  Serial.println("[DONE] Movement complete.");
}

// === Emergency Stop ===
void emergencyStop() {
  analogWrite(PWM_PIN, 0);
  delay(200);
  Serial.println("[STOP] Motor stopped.");
}
