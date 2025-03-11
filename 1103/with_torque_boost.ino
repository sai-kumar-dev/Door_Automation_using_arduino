// === AUTOMATIC DOOR CONTROL SYSTEM ===
// With encoder-based motion, button edge detection, auto-close logic, torque boosting, and safe edge deceleration

#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define HOME 11

volatile int encoderCount = 0;
const int maxPosition = 1050;
const int baseCloseDelay = 5000; // 5 seconds auto-close
const unsigned long maxMotionDuration = 7000; // Max time (ms) allowed for open/close movement

bool isOpening = false;
bool isClosing = false;
bool homingDone = false;
unsigned long closeStartTime = 0;
unsigned long closeDelay = baseCloseDelay;
unsigned long motionStartTime = 0;
int currentSpeed = 0;

// Button state tracking for edge detection
bool prevOpenState = HIGH;
bool prevCloseState = HIGH;

void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

int calculateSpeed(int position) {
  int minSpeed = 80;
  int maxSpeed = 180;
  int slowZone = maxPosition * 0.2;
  int mediumZoneEnd = maxPosition * 0.8;
  int decelerationZone = maxPosition * 0.05;

  // Torque boost zones for edge cases
  if (isClosing && position >= maxPosition * 0.95 && position < maxPosition - decelerationZone) {
    return 200; // Strong boost near close end but before soft zone
  }
  if (isOpening && position <= maxPosition * 0.05 && position > decelerationZone) {
    return 180; // Mild boost near open end but before soft zone
  }

  // Soft landing near the final few ticks
  if ((isClosing && position >= maxPosition - decelerationZone) ||
      (isOpening && position <= decelerationZone)) {
    return 100; // Slow final approach speed to avoid glass stress
  }

  int targetSpeed = (position < slowZone || position > mediumZoneEnd) ? minSpeed : maxSpeed;
  int speedStep = 5;
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
  Serial.println("🔌 System Ready. Waiting for Homing...");
}

void loop() {
  // Homing routine (door open reference)
  if (!homingDone) {
    if (digitalRead(HOME) == HIGH) {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, 80);
    } else {
      stopMotor();
      encoderCount = 0;
      homingDone = true;
      Serial.println("✅ Homing Complete! Door is Fully Opened.");
      closeStartTime = millis();
    }
    return;
  }

  // === Button Edge Detection ===
  bool currOpenState = digitalRead(OPEN_BUTTON);
  bool currCloseState = digitalRead(CLOSE_BUTTON);

  if (prevOpenState == HIGH && currOpenState == LOW) {
    // Open button pressed
    if (encoderCount <= 0) {
      Serial.println("ℹ️ Door is already fully open.");
    } else {
      isOpening = true;
      isClosing = false;
      motionStartTime = millis();
      Serial.println("🔼 Opening door...");
    }
  }

  if (prevCloseState == HIGH && currCloseState == LOW) {
    // Close button pressed
    if (encoderCount >= maxPosition) {
      Serial.println("ℹ️ Door is already fully closed.");
    } else {
      isClosing = true;
      isOpening = false;
      motionStartTime = millis();
      Serial.println("🔒 Closing door...");
    }
  }

  prevOpenState = currOpenState;
  prevCloseState = currCloseState;

  // === OPENING ACTION ===
  if (isOpening) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(HIGH);

    if (encoderCount <= 0) {
      stopMotor();
      isOpening = false;
      closeStartTime = millis();
      Serial.println("✅ Door Fully Opened. Auto-close Timer Started.");
    } else if (millis() - motionStartTime > maxMotionDuration) {
      stopMotor();
      isOpening = false;
      Serial.println("⚠️ Opening Timeout! Motion stopped to prevent damage.");
    }
  }

  // === CLOSING ACTION ===
  if (isClosing) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(LOW);

    if (encoderCount >= maxPosition) {
      stopMotor();
      isClosing = false;
      closeDelay = baseCloseDelay;
      Serial.println("🔒 Door Fully Closed. System Ready.");
    } else if (millis() - motionStartTime > maxMotionDuration) {
      stopMotor();
      isClosing = false;
      Serial.println("⚠️ Closing Timeout! Motion stopped to prevent damage.");
    }
  }

  // === AUTO-CLOSE AFTER OPEN ===
  if (!isOpening && !isClosing && encoderCount <= 0 && (millis() - closeStartTime > closeDelay)) {
    isClosing = true;
    motionStartTime = millis();
    Serial.println("⏳ Auto-Closing Initiated...");
  }
}
