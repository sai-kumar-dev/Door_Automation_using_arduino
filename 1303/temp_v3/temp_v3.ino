// Auto Door System - Optimized & Annotated Version
// Author: You 🧠 | Assistant: ChatGPT 🤖
// Purpose: Smart door control with encoder-based homing, smooth motor control, and safety logic

// ------------ PIN SETUP ------------
const int dirPin = 8;
const int pwmPin = 9;
int SPEED_STEP = 5;
const int openBtn = 5;
const int closeBtn = 6;
const int homeSensor = 11;

const int encoderA = 3;
const int encoderB = 4;

// ------------ CONFIGURATION ------------
const int minSpeed = 40;
const int maxSpeed = 120;
const int speedStep = 5;
const int maxPosition = -1030 ;    // Maximum encoder count for full close

// ------------ STATE VARIABLES ------------
volatile long encoderCount = 0;
int currentSpeed = minSpeed;
bool movementDetected = false;
unsigned long lastSpeedIncrementTime = 0;
bool isOpening = false;
bool isClosing = false;
bool homed = false;
bool debug = true; // Set to false to hide serial prints


// ------------ SETUP ------------
void setup() {
  Serial.begin(9600);
  
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  pinMode(openBtn, INPUT_PULLUP);
  pinMode(closeBtn, INPUT_PULLUP);
  pinMode(homeSensor, INPUT_PULLUP);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);

  delay(5000); // delay of 5 seconds to avoid rushing and preventing unwanted movement

  if (debug) Serial.println("System Initializing...");

  startHoming(); // Begin with door opening
}

// ------------ LOOP ------------
void loop() {
  if (!homed) return;  // Ignore button input until homing complete

  showStatus(); // Always show encoder & speed

  // --- Button Checks ---
  if (digitalRead(openBtn) == LOW) handleOpenButton();
  else if (digitalRead(closeBtn) == LOW) handleCloseButton();

  // --- Smooth Speed Control during Opening ---
  if (isOpening) {
    if (digitalRead(homeSensor) == LOW || encoderCount >= 0) {
      stopMotor();
      if (debug) Serial.println("Door fully opened. Home position reached.");
      encoderCount = 0;
      isOpening = false;
    } else {
      changeSpeed(1);
    }
  }

  // --- Smooth Speed Control during Closing ---
  if (isClosing) {
    if (encoderCount <= maxPosition) {
      stopMotor();
      if (debug) Serial.println("Door fully closed.");
      isClosing = false;
    } else {
      changeSpeed(-1);
    }
  }

  delay(20);
}

// ------------ ENCODER ISR ------------
void encoderISR() {
  if (digitalRead(encoderA) > digitalRead(encoderB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ------------ HOMING FUNCTION ------------
void startHoming() {
  if (debug) Serial.println("Homing in progress...");
  digitalWrite(dirPin, HIGH); // Open direction
  analogWrite(pwmPin, minSpeed);
  currentSpeed = minSpeed;
  isOpening = true;

  while (digitalRead(homeSensor) != LOW) {
    changeSpeed(1);
    showStatus();
    delay(20);
  }

  stopMotor();
  encoderCount = 0;
  homed = true;
  isOpening = false;

  if (debug) Serial.println("Homing complete.");
}

// ------------ BUTTON HANDLERS ------------
void handleOpenButton() {
  if (encoderCount >= 0) {
    if (debug) Serial.println("Door is already open.");
    return;
  }

  if (isClosing) {
    if (debug) Serial.println("Emergency stop during closing!");
    stopMotor();
    startOpening();
    return;
  }

  if (!isOpening) {
    if (debug) Serial.println("Manual open requested.");
    startOpening();
  }
}

void handleCloseButton() {
  if (encoderCount <= maxPosition) {
    if (debug) Serial.println("Door is already closed.");
    return;
  }

  if (isOpening) {
    if (debug) Serial.println("Emergency stop during opening!");
    stopMotor();
    return;
  }

  if (!isClosing) {
    if (debug) Serial.println("Manual close requested.");
    startClosing();
  }
}

// ------------ OPEN/CLOSE STARTERS ------------
void startOpening() {
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, minSpeed);
  currentSpeed = minSpeed;
  isOpening = true;
}

void startClosing() {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, minSpeed);
  currentSpeed = minSpeed;
  isClosing = true;
}

// ------------ MOTOR CONTROL ------------
void stopMotor() {
  analogWrite(pwmPin, 0);
  currentSpeed = minSpeed;
  isOpening = false;
  isClosing = false;
}

// ------------ DEBUG STATUS DISPLAY ------------
void showStatus() {
  if (debug) {
    Serial.print("Encoder: ");
    Serial.print(encoderCount);
    Serial.print(" | Speed: ");
    Serial.println(currentSpeed);
  }
}

void changeSpeed(int direction) {
  // Ramp-up phase before movement detection
  if (!movementDetected) {
    if (encoderCount != 0 || encoderCount != maxPosition) { // Check for movement detection
      movementDetected = true;
    } else {
      // Increment speed at regular intervals
      if (millis() - lastSpeedIncrementTime >= 1000) {
        currentSpeed += SPEED_STEP;
        currentSpeed = constrain(currentSpeed, minSpeed, maxSpeed); // Ensure speed is within bounds
        lastSpeedIncrementTime = millis();
      }
    }
  }

  // Deceleration phase after movement detection
  if (movementDetected) {
    // Calculate the encoder count at which deceleration starts
    int decreaseStartEncoder = (direction == 1) ? 0.2 * maxPosition  : -0.8 * maxPosition;
    // Calculate the encoder count at which the motor should reach minSpeed
    int endEncoder = (direction == 1) ? 0 : maxPosition;

    // Set speed to maxSpeed until deceleration starts
    if ((direction == 1 && encoderCount <= decreaseStartEncoder) || (direction == -1 && encoderCount > = decreaseStartEncoder)) {
      analogWrite(pwmPin, maxSpeed);
    }
    // Set speed to minSpeed when the encoder reaches the end position
    else if ((direction == 1 && encoderCount <= endEncoder) || (direction == -1 && encoderCount >= endEncoder)) {
      analogWrite(pwmPin, minSpeed);
    }
    // Calculate and set the decelerating speed
    else {
      currentSpeed = calculateDeceleration(encoderCount, decreaseStartEncoder, endEncoder);
      analogWrite(pwmPin, currentSpeed);
    }
  }
}

int calculateDeceleration(long encoderCount, int decreaseStartEncoder, int endEncoder) {
  float speedRange = maxSpeed - minSpeed;
  float encoderRange = abs(decreaseStartEncoder - endEncoder);
  float encoderProgress = abs(encoderCount - decreaseStartEncoder);
  float speedDecrease = (encoderProgress / encoderRange) * speedRange;
  int currentSpeed = maxSpeed - speedDecrease;
  return constrain(currentSpeed, minSpeed, maxSpeed);
}
