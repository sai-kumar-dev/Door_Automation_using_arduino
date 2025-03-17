// Pin Definitions
#define ENCODER_A 3
#define ENCODER_B 4
#define DIR_PIN 8
#define PWM_PIN 9
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define PIR_SENSOR 12
#define HOME_SENSOR 11

// Encoder and Motor Parameters
volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastEncoderMovementTime = 0;
#define MAX_ENCODER_COUNT 1000
#define HOMING_SPEED 120
#define MAX_SPEED 255
#define MIN_SPEED 80
#define ACCEL_DISTANCE 600
#define AUTO_CLOSE_DELAY 5000

// Stall Detection
#define ENCODER_STALL_TIMEOUT 200
#define STALL_BOOST_SPEED 255
#define STALL_RECOVERY_DURATION 500
bool isStallBoostActive = false;
unsigned long stallBoostStartTime = 0;

// State Definitions
enum State { INIT, HOMING, IDLE, OPENING, CLOSING };
State state = INIT;

bool isHomed = false;
unsigned long autoCloseTimer = 0;

// Function Prototypes
void updateEncoder();
void moveMotor(bool dir, int speed);
void stopMotor();
int calculateSpeed(long currentPos, long targetPos);
void handleButtons();
void handleMotionSensor();
void checkStallLogic(long targetPos);
void transitionTo(State newState);
void doOpening();
void doClosing();

void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  pinMode(PIR_SENSOR, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);

  transitionTo(HOMING);
}

void loop() {
  handleButtons();
  handleMotionSensor();

  switch (state) {
    case HOMING:
      moveMotor(true, HOMING_SPEED);
      if (digitalRead(HOME_SENSOR) == LOW) {
        stopMotor();
        encoderCount = 0;
        isHomed = true;
        Serial.println("Homing complete. Encoder reset.");
        transitionTo(IDLE);
        autoCloseTimer = millis() + AUTO_CLOSE_DELAY;
      }
      break;

    case OPENING:
      doOpening();
      break;

    case CLOSING:
      doClosing();
      break;

    case IDLE:
      if (isHomed && millis() > autoCloseTimer) {
        transitionTo(CLOSING);
      }
      break;

    default:
      break;
  }
}

// === Encoder ISR ===
void updateEncoder() {
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? 1 : -1;
}

void moveMotor(bool dir, int speed) {
  digitalWrite(DIR_PIN, dir);
  analogWrite(PWM_PIN, constrain(speed, 0, 255));
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
}

int calculateSpeed(long currentPos, long targetPos) {
  if (isStallBoostActive) return STALL_BOOST_SPEED;

  long distance = abs(targetPos - currentPos);
  float brakingThreshold = 0.6 * abs(targetPos - (targetPos > 0 ? MAX_ENCODER_COUNT : -MAX_ENCODER_COUNT));

  if (distance >= brakingThreshold)
    return MAX_SPEED;

  float fraction = (float)distance / brakingThreshold;
  float speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * pow(fraction, 2.5); // exponential deceleration
  return constrain((int)speed, MIN_SPEED, MAX_SPEED);
}

void handleButtons() {
  if (!isHomed) return;

  if (digitalRead(OPEN_BUTTON) == LOW) {
    if (state != OPENING) transitionTo(OPENING);
    autoCloseTimer = millis() + AUTO_CLOSE_DELAY;
  }

  if (digitalRead(CLOSE_BUTTON) == LOW) {
    if (state != CLOSING) transitionTo(CLOSING);
  }
}

void handleMotionSensor() {
  if (!isHomed || state == HOMING) return;

  if (digitalRead(PIR_SENSOR) == HIGH) {
    if (state != OPENING) transitionTo(OPENING);
    autoCloseTimer = millis() + AUTO_CLOSE_DELAY;
  }
}

void checkStallLogic(long targetPos) {
  if (encoderCount != lastEncoderCount) {
    lastEncoderCount = encoderCount;
    lastEncoderMovementTime = millis();
    if (isStallBoostActive) {
      isStallBoostActive = false;
      Serial.println("Stall cleared.");
    }
  } else if (!isStallBoostActive && millis() - lastEncoderMovementTime > ENCODER_STALL_TIMEOUT) {
    isStallBoostActive = true;
    stallBoostStartTime = millis();
    Serial.println("Stall detected! Boosting speed...");
  }

  if (isStallBoostActive && millis() - stallBoostStartTime > STALL_RECOVERY_DURATION) {
    isStallBoostActive = false;
    Serial.println("Stall boost timeout.");
  }
}

void transitionTo(State newState) {
  stopMotor();
  state = newState;
  Serial.print("State changed to: ");
  Serial.println(newState);
}

void doOpening() {
  long target = MAX_ENCODER_COUNT;
  checkStallLogic(target);
  int speed = calculateSpeed(encoderCount, target);
  moveMotor(false, speed);
  if (encoderCount >= target) {
    stopMotor();
    transitionTo(IDLE);
    autoCloseTimer = millis() + AUTO_CLOSE_DELAY;
  }
}

void doClosing() {
  long target = 0;
  checkStallLogic(target);
  int speed = calculateSpeed(encoderCount, target);
  moveMotor(true, speed);
  if (encoderCount <= target) {
    stopMotor();
    transitionTo(IDLE);
  }
}
