// Auto Door System - Optimized & Annotated Version
// Author: You 🧠 | Assistant: ChatGPT 🤖
// Purpose: Smart door control with encoder-based homing, smooth motor control, and safety logic

// ------------ PIN SETUP ------------
const int dirPin = 8;
const int pwmPin = 9;

const int openBtn = 5;
const int closeBtn = 6;
const int homeSensor = 11;

const int encoderA = 3;
const int encoderB = 4;

// ------------ CONFIGURATION ------------
const int minSpeed = 80;
const int maxSpeed = 120;
const int speedStep = 5;
const int autoCloseDelay = 5000;  // Delay before auto-close (ms)
const int maxPosition = -20;    // Maximum encoder count for full close

// ------------ STATE VARIABLES ------------
volatile long encoderCount = 0;
int currentSpeed = minSpeed;
bool isOpening = false;
bool isClosing = false;
bool homed = false;
bool debug = true; // Set to false to hide serial prints

unsigned long openStartTime = 0;

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

  // --- Auto-Close Logic ---
  if (openStartTime > 0 && millis() - openStartTime >= autoCloseDelay) {
    if (debug) Serial.println("Auto-closing after timeout...");
    startClosing();
    openStartTime = 0;
  }

  // --- Smooth Speed Control during Opening ---
  if (isOpening) {
    if (digitalRead(homeSensor) == LOW || encoderCount >= 0) {
      stopMotor();
      if (debug) Serial.println("Door fully opened. Home position reached.");
      encoderCount = 0;
      openStartTime = millis(); // Start auto-close timer
      isOpening = false;
    } else {
      smoothDrive();
    }
  }

  // --- Smooth Speed Control during Closing ---
  if (isClosing) {
    if (encoderCount <= maxPosition) {
      stopMotor();
      if (debug) Serial.println("Door fully closed.");
      isClosing = false;
    } else {
      smoothDrive();
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
    smoothDrive();
    showStatus();
    delay(20);
  }

  stopMotor();
  encoderCount = 0;
  homed = true;
  isOpening = false;

  if (debug) Serial.println("Homing complete.");
  delay(autoCloseDelay); // wait before auto-closing
  startClosing();
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
  openStartTime = 0;
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

void smoothDrive() {
  // Gradually ramp up speed if door is stuck
  if (currentSpeed < maxSpeed) currentSpeed += speedStep;
  else currentSpeed = maxSpeed;

  analogWrite(pwmPin, currentSpeed);
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
