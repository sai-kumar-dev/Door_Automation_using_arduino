// Arduino-based Automatic Door Control System
// Supports a single Open button for operation and an additional Close button for manual override.
// The door opens when the Open button is pressed and automatically closes after a delay.
// Pressing Open again during opening increases the delay before closing.
// Pressing Open while closing retracts the door (opens again).
// The Close button allows manual closing anytime.

#define ENCODER_A 3  // Encoder A pin
#define ENCODER_B 4  // Encoder B pin
#define PWM_PIN 9    // PWM pin for motor speed control
#define DIR_PIN 8    // Direction pin for motor
#define OPEN_BUTTON 5 // Open button pin
#define CLOSE_BUTTON 6 // Close button pin

volatile int encoderCount = 0;  // Tracks door position
const int maxPosition = 1000;    // Maximum open position
const int baseCloseDelay = 5000; // Base delay before auto-closing (5 seconds)

bool isOpening = false;  // Indicates if the door is opening
bool isClosing = false;  // Indicates if the door is closing
unsigned long closeStartTime = 0; // Time when the door was fully opened
unsigned long closeDelay = baseCloseDelay; // Adjustable delay before auto-closing

int currentSpeed = 0; // Tracks the motor speed for smooth acceleration/deceleration

// Interrupt Service Routine (ISR) for encoder to track door movement
void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Function to calculate motor speed based on door position
int calculateSpeed(int position) {
  int minSpeed = 80;  // Minimum speed for smooth operation
  int maxSpeed = 180; // Maximum speed
  int slowZone = maxPosition * 0.2;  // Zone near start and end for slow movement
  int mediumZoneEnd = maxPosition * 0.8;
  
  int targetSpeed = (position < slowZone || position > mediumZoneEnd) ? minSpeed : maxSpeed;

  // Gradual speed change to avoid sudden jerks
  int speedStep = 5;
  if (currentSpeed < targetSpeed) {
    currentSpeed = min(currentSpeed + speedStep, targetSpeed);
  } else if (currentSpeed > targetSpeed) {
    currentSpeed = max(currentSpeed - speedStep, targetSpeed);
  }
  
  return currentSpeed;
}

// Function to move the motor in a given direction
void moveMotor(int direction) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, currentSpeed);
}

// Function to stop the motor
void stopMotor() {
  analogWrite(PWM_PIN, 0);
  currentSpeed = 0;
}

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_BUTTON, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
}

void loop() {
  // Open button logic
  if (digitalRead(OPEN_BUTTON) == HIGH) {
    if (isClosing) {
      isClosing = false;
      isOpening = true;
      Serial.println("Retracting - Opening again...");
    } else {
      if (!isOpening) {
        isOpening = true;
        Serial.println("Opening...");
      } else {
        closeDelay += 5000; // Increase auto-close delay by 5 seconds
        Serial.print("Extending delay: ");
        Serial.println(closeDelay);
      }
    }
  }

  // Close button logic for manual override
  if (digitalRead(CLOSE_BUTTON) == HIGH) {
    if (isOpening) {
      isOpening = false;
      isClosing = true;
      Serial.println("Manual Close Triggered...");
    } else if (!isClosing) {
      isClosing = true;
      Serial.println("Closing...");
    }
  }

  // Motor control logic for opening
  if (isOpening) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(HIGH);
    if (abs(encoderCount) >= maxPosition) {
      stopMotor();
      isOpening = false;
      closeStartTime = millis();
      Serial.println("Door Fully Open");
    }
    Serial.print("Opening, Encoder: ");
    Serial.println(encoderCount);
  }

  // Auto-close trigger after delay if no input received
  if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
    isClosing = true;
    Serial.println("Closing...");
  }

  // Motor control logic for closing
  if (isClosing) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(LOW);
    if (encoderCount <= 0) {
      stopMotor();
      isClosing = false;
      closeDelay = baseCloseDelay; // Reset delay after full close
      Serial.println("Door Fully Closed");
    }
    Serial.print("Closing, Encoder: ");
    Serial.println(encoderCount);
  }
}
