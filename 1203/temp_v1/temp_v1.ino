#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define HOME 11

volatile int encoderCount = 0;
const int maxPosition = 300;

bool homed = false;
bool isOpening = false;
bool isClosing = false;
bool emergencyStopped = false;

int currentSpeed = 0;

void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

int calculateSpeed(int position, bool isOpeningDir) {
  int minSpeed = 60;
  int maxSpeed = 120;
  int slowZone = maxPosition * 0.05;
  int fastZoneStart = maxPosition * 0.2;
  int fastZoneEnd = maxPosition * 0.95;

  int targetSpeed = minSpeed;

  if (position > fastZoneStart && position < fastZoneEnd) {
    targetSpeed = maxSpeed;
  } else {
    targetSpeed = minSpeed;
  }

  // Torque boost at final end zones
  if (!isOpeningDir && position >= maxPosition * 0.95) {
    targetSpeed = 200;
  }
  if (isOpeningDir && position <= maxPosition * 0.05) {
    targetSpeed = 180;
  }

  // Gradual ramping
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
  Serial.print("Encoder Count :");
  Serial.println(encoderCount);
  if (!homed) {
    if (digitalRead(HOME) == LOW) {
      stopMotor();
      encoderCount = 0;
      homed = true;
      Serial.println("✅ Homing Complete. Door is Fully Open.");
    } else {
      /*
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, 100);
      */
    }
    return;
  }

  bool openPressed = digitalRead(OPEN_BUTTON) == LOW;
  bool closePressed = digitalRead(CLOSE_BUTTON) == LOW;

  // Emergency stop case
  if ((isOpening || isClosing) && openPressed && closePressed) {
    stopMotor();
    isOpening = false;
    isClosing = false;
    emergencyStopped = true;
    Serial.println("🛑 Emergency Stop: Both Buttons Pressed!");
    return;
  }

  if (openPressed && !isOpening && !isClosing) {
    if (encoderCount >= 0) {
      Serial.println("ℹ️ Door is already open.");
    } else {
      isOpening = true;
      Serial.println("🔼 Opening Door...");
    }
  }

  if (closePressed && !isOpening && !isClosing) {
    if (abs(encoderCount) >= maxPosition) {
      Serial.println("ℹ️ Door is already closed.");
    } else {
      isClosing = true;
      Serial.println("🔽 Closing Door...");
    }
  }

  if (isOpening) {
    int speed = calculateSpeed(abs(encoderCount), true);
    moveMotor(HIGH);
    if (encoderCount >= 0 ) {
      stopMotor();
      isOpening = false;
      Serial.println("✅ Door Fully Opened.");
    }
  }

  if (isClosing) {
    int speed = calculateSpeed(abs(encoderCount), false);
    moveMotor(LOW);
    if (abs(encoderCount) >= maxPosition) {
      stopMotor();
      isClosing = false;
      Serial.println("🔒 Door Fully Closed.");
    }
  }
}
