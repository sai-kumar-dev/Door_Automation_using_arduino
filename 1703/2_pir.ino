// === Pin Definitions ===
#define ENCODER_A 3
#define ENCODER_B 4
#define HOME_SENSOR 11
#define OPEN_BTN 5
#define CLOSE_BTN 6
#define DIR_PIN 8
#define PWM_PIN 9
#define PIR_SENSOR_1 12
#define PIR_SENSOR_2 13

// === Motor Control Constants ===
#define MAX_SPEED 180
#define MIN_SPEED 50
#define BOOST_SPEED 200
#define STALL_THRESHOLD 200    // ms without encoder change
#define AUTO_CLOSE_DELAY 5000  // ms delay before auto close

// === Door Position Constants ===
#define CLOSED_POSITION 1000
#define OPEN_POSITION 0

// === State Variables ===
volatile int encoderCount = 0;
unsigned long lastEncoderChange = 0;
bool isHomed = false;
bool isOpening = false;
bool isClosing = false;
bool pirTriggered = false;
unsigned long lastOpenTime = 0;

// === Setup ===
void setup() {
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(PIR_SENSOR_1, INPUT_PULLUP);
  pinMode(PIR_SENSOR_2, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);
  Serial.begin(115200);
  delay(2000);
  Serial.println("Booting... Starting homing...");
  homeDoor();
}

// === Loop ===
void loop() {
  Serial.print("ENC: ");
  Serial.println(encoderCount);

  if (!isHomed) return;

  // PIR or Button triggered open
  if ((digitalRead(PIR_SENSOR_1) == LOW || digitalRead(PIR_SENSOR_2) == LOW) && !pirTriggered) {
    pirTriggered = true;
    handleOpen();
  } else if (digitalRead(PIR_SENSOR_1) == HIGH && digitalRead(PIR_SENSOR_2) == HIGH) {
    pirTriggered = false;
  }

  if (digitalRead(OPEN_BTN) == LOW) {
    delay(100);
    if (digitalRead(OPEN_BTN) == LOW) handleOpen();
  }

  if (digitalRead(CLOSE_BTN) == LOW) {
    delay(100);
    if (digitalRead(CLOSE_BTN) == LOW) handleClose();
  }

  // Auto-close logic
  if (encoderCount == OPEN_POSITION && millis() - lastOpenTime > AUTO_CLOSE_DELAY && !isClosing) {
    handleClose();
  }
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) == digitalRead(ENCODER_A)) ? 1 : -1;
  encoderCount = constrain(encoderCount, OPEN_POSITION, CLOSED_POSITION);
  lastEncoderChange = millis();
}

// === Homing ===
void homeDoor() {
  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 100);
  while (digitalRead(HOME_SENSOR) == HIGH) {
    delay(50);
  }
  analogWrite(PWM_PIN, 0);
  encoderCount = 0;
  isHomed = true;
  Serial.println("Homed. Encoder = 0");
}

// === Open Door ===
void handleOpen() {
  if (isOpening || encoderCount == OPEN_POSITION) return;
  Serial.println("[ACTION] Opening...");
  isOpening = true;
  isClosing = false;
  moveDoor(LOW, OPEN_POSITION);
  encoderCount = OPEN_POSITION;
  lastOpenTime = millis();
  isOpening = false;
}

// === Close Door ===
void handleClose() {
  if (isClosing || encoderCount == CLOSED_POSITION) return;
  Serial.println("[ACTION] Closing...");
  isClosing = true;
  isOpening = false;
  moveDoor(HIGH, CLOSED_POSITION);
  encoderCount = CLOSED_POSITION;
  isClosing = false;
}

// === Motor Control with Exponential Speed ===
void moveDoor(bool dir, int target) {
  digitalWrite(DIR_PIN, dir);
  int speed = MAX_SPEED;
  int initial = encoderCount;
  int totalDist = abs(target - initial);
  int brakingStart = initial + 0.6 * (target - initial);

  while ((dir == HIGH && encoderCount < target) || (dir == LOW && encoderCount > target)) {
    int distRemaining = abs(target - encoderCount);
    if ((dir == HIGH && digitalRead(OPEN_BTN) == LOW) || (dir == LOW && digitalRead(CLOSE_BTN) == LOW)) {
      Serial.println("[EMERGENCY] Stop triggered.");
      analogWrite(PWM_PIN, 0);
      delay(200);
      return;
    }

    // Encoder stall handling
    if (millis() - lastEncoderChange > STALL_THRESHOLD) {
      Serial.println("[STALL] Encoder stuck. Boosting...");
      analogWrite(PWM_PIN, BOOST_SPEED);
      continue;
    }

    // Dynamic speed (exponential-like slowdown)
    float ratio = (float)distRemaining / totalDist;
    speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * pow(ratio, 2); // Exponential falloff
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    analogWrite(PWM_PIN, speed);
    Serial.print("ENC: ");
    Serial.print(encoderCount);
    Serial.print(" | SPD: ");
    Serial.println(speed);
    delay(30);
  }

  analogWrite(PWM_PIN, 0);
  Serial.println("[DONE] Motion complete.");
}
