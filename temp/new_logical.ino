// Pin Definitions
#define DIR_PIN       2
#define PWM_PIN       3
#define ENCODER_PIN   4
#define HOME_SENSOR   5
#define OPEN_BTN      6
#define CLOSE_BTN     7

// Motor Control
#define MIN_SPEED     70
#define MAX_SPEED     200
#define SPEED_STEP    5

// Encoder Logic
volatile long encoderCount = 0;
long maxPosition = -1000; // hypothetical maximum closed position

// States
enum DoorState {HOMING, IDLE, OPENING, CLOSING, STOPPED};
DoorState doorState = HOMING;

bool motorRunning = false;
unsigned long lastMoveTime = 0;
unsigned long autoCloseDelay = 5000;

// Function declarations
void setMotor(bool direction, int speed);
void stopMotor(String reason);
void updateMotorSpeed();
void printDebug();
bool isHomeSensorTriggered();
bool isButtonPressed(int pin);

// Encoder ISR (simple simulation)
void encoderISR() {
  if (digitalRead(DIR_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  
  Serial.println("System Initializing...");

  // Start homing
  setMotor(true, MIN_SPEED);
  doorState = HOMING;
}

void loop() {
  printDebug();

  switch (doorState) {
    case HOMING:
      if (isHomeSensorTriggered()) {
        encoderCount = 0;
        stopMotor("Homed");
        delay(1000);
        doorState = IDLE;
        lastMoveTime = millis();
      }
      break;

    case IDLE:
      if (millis() - lastMoveTime > autoCloseDelay) {
        Serial.println("Auto-closing after delay");
        doorState = CLOSING;
        setMotor(false, MIN_SPEED);
      } else {
        if (isButtonPressed(OPEN_BTN)) {
          Serial.println("Opening requested");
          if (encoderCount >= 0) Serial.println("Already open!");
          else {
            doorState = OPENING;
            setMotor(true, MIN_SPEED);
          }
        }
        if (isButtonPressed(CLOSE_BTN)) {
          Serial.println("Closing requested");
          doorState = CLOSING;
          setMotor(false, MIN_SPEED);
        }
      }
      break;

    case OPENING:
      updateMotorSpeed();
      if (isHomeSensorTriggered()) {
        stopMotor("Fully Opened");
        doorState = IDLE;
        lastMoveTime = millis();
      }
      else if (isButtonPressed(CLOSE_BTN)) {
        stopMotor("Emergency Stop during Opening");
        doorState = STOPPED;
      }
      break;

    case CLOSING:
      updateMotorSpeed();
      if (encoderCount <= maxPosition) {
        stopMotor("Fully Closed");
        doorState = IDLE;
        lastMoveTime = millis();
      }
      else if (isButtonPressed(OPEN_BTN)) {
        stopMotor("Stop & Open during Closing");
        delay(300);
        doorState = OPENING;
        setMotor(true, MIN_SPEED);
      }
      else if (isButtonPressed(CLOSE_BTN)) {
        Serial.println("Already closing!");
      }
      break;

    case STOPPED:
      if (isButtonPressed(OPEN_BTN)) {
        Serial.println("Resuming Opening...");
        doorState = OPENING;
        setMotor(true, MIN_SPEED);
      }
      if (isButtonPressed(CLOSE_BTN)) {
        Serial.println("Resuming Closing...");
        doorState = CLOSING;
        setMotor(false, MIN_SPEED);
      }
      break;
  }

  delay(50);  // reduce clutter
}

//============================//
//         FUNCTIONS          //
//============================//

void setMotor(bool direction, int speed) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, speed);
  motorRunning = true;
}

void stopMotor(String reason) {
  analogWrite(PWM_PIN, 0);
  motorRunning = false;
  Serial.println("Motor Stopped: " + reason);
}

bool isHomeSensorTriggered() {
  return digitalRead(HOME_SENSOR) == LOW;
}

bool isButtonPressed(int pin) {
  return digitalRead(pin) == LOW;
}

void updateMotorSpeed() {
  static int currentSpeed = MIN_SPEED;
  static long lastEncoder = encoderCount;
  static unsigned long lastUpdate = millis();

  if (!motorRunning) return;

  // No movement? Bump speed
  if (encoderCount == lastEncoder && currentSpeed < MAX_SPEED) {
    currentSpeed += SPEED_STEP;
    setMotor(digitalRead(DIR_PIN), currentSpeed);
  }
  // Approaching target? Slow down
  else if (encoderCount != lastEncoder && currentSpeed > MIN_SPEED) {
    currentSpeed -= SPEED_STEP;
    setMotor(digitalRead(DIR_PIN), currentSpeed);
  }

  lastEncoder = encoderCount;
  lastUpdate = millis();
}

void printDebug() {
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 250) {
    Serial.print("Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(motorRunning ? analogRead(PWM_PIN) : 0);
    lastDebugTime = millis();
  }
}
