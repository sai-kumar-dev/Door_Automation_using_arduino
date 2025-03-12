#include <Arduino.h>

#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define HOME 11

volatile int encoderCount = 0;
const int maxPosition = 1050;
const int baseCloseDelay = 5000;

bool homed = false;
bool isOpening = false;
bool isClosing = false;
bool emergencyStopped = false;
unsigned long closeStartTime = 0;
unsigned long closeDelay = baseCloseDelay;
unsigned long motionStartTime = 0;
const unsigned long maxMotionDuration = 7000; // 7 seconds timeout safety

int currentSpeed = 0;
bool lastOpenState = HIGH;
bool lastCloseState = HIGH;

void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

int calculateSpeed(int position, bool isOpeningDir) {
  int maxSpeed = 100;
  int minSpeed = 30;
  int rampStartZone = 150;
  int rampActivationThreshold = 50;

  int distanceToEnd = isOpeningDir ? position : maxPosition - position;
  int targetSpeed = maxSpeed;

  if (distanceToEnd <= rampStartZone && distanceToEnd < (maxPosition - rampActivationThreshold)) {
    float rampRatio = (float)(distanceToEnd - rampActivationThreshold) / (rampStartZone - rampActivationThreshold);
    rampRatio = constrain(rampRatio, 0.0, 1.0);
    targetSpeed = minSpeed + (maxSpeed - minSpeed) * rampRatio;
  }

  int speedStep = 3;
  if (currentSpeed < targetSpeed) {
    currentSpeed = min(currentSpeed + speedStep, targetSpeed);
  } else if (currentSpeed > targetSpeed) {
    currentSpeed = max(currentSpeed - speedStep, targetSpeed);
  }

  return currentSpeed;
}

void moveMotor(int direction) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, currentSpeed);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  currentSpeed = 0;
}

bool performSmartHoming() {
  const int initialSpeed = 30;
  const int maxHomingSpeed = 100;
  const int speedStep = 10;
  const int minSafeSpeed = 25;
  int testSpeed = initialSpeed;
  int previousEncoder = encoderCount;
  unsigned long lastCheck = millis();
  unsigned long checkInterval = 300;
  unsigned long homingStartTime = millis();
  const unsigned long homingTimeout = 10000; // 10 seconds

  Serial.println("🏠 Starting Smart Homing...");
  
  while (digitalRead(HOME) != LOW) {
    analogWrite(PWM_PIN, testSpeed);
    digitalWrite(DIR_PIN, HIGH);

    if (millis() - homingStartTime > homingTimeout) {
      stopMotor();
      Serial.println("❌ Homing Timeout! Sensor not triggered.");
      return false; // Failed to home
    }

    if (millis() - lastCheck >= checkInterval) {
      int delta = abs(encoderCount - previousEncoder);
      if (delta < 2 && testSpeed < maxHomingSpeed) {
        testSpeed = min(testSpeed + speedStep, maxHomingSpeed);
        Serial.print("⏫ Increasing Homing Speed to: ");
        Serial.println(testSpeed);
      } else if (delta >= 2 && testSpeed > minSafeSpeed) {
        testSpeed = max(testSpeed - speedStep, minSafeSpeed);
        Serial.print("⏬ Slowing Homing Speed to: ");
        Serial.println(testSpeed);
      }
      previousEncoder = encoderCount;
      lastCheck = millis();
    }
  }

  stopMotor();
  encoderCount = 0;
  Serial.println("✅ Homing Complete. Door is Fully Open.");
  return true;
}


void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(HOME, INPUT_PULLUP);
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  delay(1000);
  Serial.println("🔌 System Ready. Waiting for Homing...");
}

void loop() {
  bool currentOpenState = digitalRead(OPEN_BUTTON);
  bool currentCloseState = digitalRead(CLOSE_BUTTON);

  bool openJustPressed = (lastOpenState == HIGH && currentOpenState == LOW);
  bool closeJustPressed = (lastCloseState == HIGH && currentCloseState == LOW);

  lastOpenState = currentOpenState;
  lastCloseState = currentCloseState;

  if (!homed) {
    homed = performSmartHoming();
    closeStartTime = millis();
    return;
  }

  bool openPressed = currentOpenState == LOW;
  bool closePressed = currentCloseState == LOW;

  if ((isOpening || isClosing) && openPressed && closePressed) {
    stopMotor();
    isOpening = false;
    isClosing = false;
    emergencyStopped = true;
    Serial.println("🛑 Emergency Stop: Both Buttons Pressed!");
    return;
  }

  if (openJustPressed && !isOpening && !isClosing) {
    if (encoderCount <= 0) {
      Serial.println("ℹ️ Door is already open.");
    } else {
      isOpening = true;
      motionStartTime = millis();
      Serial.println("🔼 Opening Door...");
    }
  }

  if (closeJustPressed && !isOpening && !isClosing) {
    if (encoderCount >= maxPosition) {
      Serial.println("ℹ️ Door is already closed.");
    } else {
      isClosing = true;
      motionStartTime = millis();
      Serial.println("🔽 Closing Door...");
    }
  }

  if (isOpening) {
    int speed = calculateSpeed(encoderCount, true);
    moveMotor(HIGH);
    if (encoderCount <= 0 || millis() - motionStartTime > maxMotionDuration) {
      stopMotor();
      isOpening = false;
      closeStartTime = millis();
      Serial.println("✅ Door Fully Opened.");
    }
  }

  if (isClosing) {
    int speed = calculateSpeed(encoderCount, false);
    moveMotor(LOW);
    if (encoderCount >= maxPosition || millis() - motionStartTime > maxMotionDuration) {
      stopMotor();
      isClosing = false;
      closeDelay = baseCloseDelay;
      Serial.println("🔒 Door Fully Closed.");
    }
  }

  // Auto-close temporarily disabled for testing
  // if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
  //   if (encoderCount < maxPosition) {
  //     isClosing = true;
  //     motionStartTime = millis();
  //     Serial.println("⏳ Auto-Close Initiated...");
  //   }
  // }
}
