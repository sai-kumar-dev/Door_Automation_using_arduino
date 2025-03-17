
/*
 * Arduino-Based Automatic Glass Door System
 * Refactored for encoder direction:
 * - Closing: 0 -> 1000
 * - Opening: 1000 -> 0
 */

// ----------------- Pin Definitions -----------------
const int pwmPin       = 9;
const int dirPin       = 8;
const int encoderA     = 3;
const int encoderB     = 4;
const int openBtnPin   = 5;
const int closeBtnPin  = 6;
const int pirPin       = 7;
const int homeSensor   = 11;

// ----------------- Control Constants -----------------
const int OPEN_POSITION     = 0;
const int CLOSED_POSITION   = 1000;
const int MAX_SPEED         = 255;
const int MIN_SPEED         = 100;
const unsigned long debounceDelay = 50;
const unsigned long autoCloseDelay = 3000;

// ----------------- Variables -----------------
volatile long encoderCount = 0;
bool homingDone = false;
bool isOpening = false;
bool isClosing = false;
unsigned long lastMotionTime = 0;
unsigned long lastOpenPressTime = 0;
unsigned long lastClosePressTime = 0;
bool autoCloseInitiated = false;
unsigned long autoCloseStartTime = 0;
long targetPosition = CLOSED_POSITION;

// ----------------- Setup -----------------
void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(openBtnPin, INPUT_PULLUP);
  pinMode(closeBtnPin, INPUT_PULLUP);
  pinMode(pirPin, INPUT);
  pinMode(homeSensor, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, CHANGE);

  Serial.begin(9600);
}

// ----------------- Loop -----------------
void loop() {
  if (!homingDone) {
    performHoming();
    return;
  }

  handleButtons();
  handlePIR();
  handleMovement();
  handleAutoClose();

  Serial.print("Encoder: ");
  Serial.println(encoderCount);
}

// ----------------- Encoder Update -----------------
void updateEncoder() {
  int A = digitalRead(encoderA);
  int B = digitalRead(encoderB);
  if (A == B) encoderCount++;
  else encoderCount--;
}

// ----------------- Homing -----------------
void performHoming() {
  digitalWrite(dirPin, HIGH); // Open direction
  analogWrite(pwmPin, 130); // Slow speed

  if (digitalRead(homeSensor) == LOW) {
    analogWrite(pwmPin, 0);
    encoderCount = OPEN_POSITION;
    homingDone = true;
    autoCloseStartTime = millis();
    autoCloseInitiated = true;
  }
}

// ----------------- Handle Button Inputs -----------------
void handleButtons() {
  if (millis() - lastOpenPressTime > debounceDelay && digitalRead(openBtnPin) == LOW) {
    lastOpenPressTime = millis();
    if (!isOpening && encoderCount < CLOSED_POSITION) {
      startOpening();
    }
  }
  if (millis() - lastClosePressTime > debounceDelay && digitalRead(closeBtnPin) == LOW) {
    lastClosePressTime = millis();
    if (!isClosing && encoderCount > OPEN_POSITION) {
      startClosing();
    }
  }
}

// ----------------- PIR Sensor -----------------
void handlePIR() {
  if (digitalRead(pirPin) == HIGH) {
    lastMotionTime = millis();
    if (!isOpening && encoderCount > OPEN_POSITION) {
      startOpening();
    }
  }
}

// ----------------- Start Movements -----------------
void startOpening() {
  isOpening = true;
  isClosing = false;
  digitalWrite(dirPin, HIGH); // OPEN direction
  targetPosition = OPEN_POSITION;
  autoCloseInitiated = false;
}

void startClosing() {
  isClosing = true;
  isOpening = false;
  digitalWrite(dirPin, LOW); // CLOSE direction
  targetPosition = CLOSED_POSITION;
}

// ----------------- Movement Handling -----------------
void handleMovement() {
  if (isOpening || isClosing) {
    long distanceToTarget = abs(targetPosition - encoderCount);
    long totalDistance = abs(CLOSED_POSITION - OPEN_POSITION);
    long brakingDistance = 0.6 * totalDistance;

    int speed = MAX_SPEED;
    if (distanceToTarget < brakingDistance) {
      float ratio = float(distanceToTarget) / brakingDistance;
      speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * pow(ratio, 2); // Exponential decay
      if (speed < MIN_SPEED) speed = MIN_SPEED;
    }

    analogWrite(pwmPin, speed);

    if ((isOpening && encoderCount <= OPEN_POSITION) ||
        (isClosing && encoderCount >= CLOSED_POSITION)) {
      analogWrite(pwmPin, 0);
      isOpening = false;
      isClosing = false;
      autoCloseStartTime = millis();
      autoCloseInitiated = true;
    }
  }
}

// ----------------- Auto Close -----------------
void handleAutoClose() {
  if (autoCloseInitiated && (millis() - autoCloseStartTime >= autoCloseDelay)) {
    if (!isClosing && encoderCount > OPEN_POSITION) {
      startClosing();
    }
    autoCloseInitiated = false;
  }
}
