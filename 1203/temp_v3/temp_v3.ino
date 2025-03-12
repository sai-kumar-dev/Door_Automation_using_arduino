#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8  
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define HOME 11
#define HOME_SPEED 80
#define MAX_SPEED 150

volatile int encoderCount = 0;
const int maxPosition = 300;

bool homed = false;
bool isOpening = false;
bool isClosing = false;
bool emergencyStopped = false;

volatile int currentSpeed = 150;

void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

int calculateSpeed() {
  currentSpeed = 10;
  return currentSpeed;
}

/*
int calculateSpeed(int position) {
  int maxSpeed = MAX_SPEED;
  int minSpeed = MIN_SPEED;
  int rampStartZone = 150;
  int rampActivationThreshold = 50;

  int distanceToEnd = maxPosition - position;
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
}*/

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
  //delay(5000);
}

void loop() {
  if (!homed) {
    if (digitalRead(HOME) == LOW) {
      stopMotor();
      encoderCount = 0;
      homed = true;
      Serial.println("✅ Homing Complete. Door is Fully Open.");
    } else {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, HOME_SPEED);
    }
    return;
  }

  bool openPressed = digitalRead(OPEN_BUTTON) == LOW;
  bool closePressed = digitalRead(CLOSE_BUTTON) == LOW;

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
      Serial.print("Opening Encoder Count :");
  Serial.println(encoderCount);
    if (digitalRead(HOME) == LOW) {
      stopMotor();
      encoderCount = 0;
      homed = true;
      Serial.println("✅Door is Fully Open.");
    } else {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, HOME_SPEED);
    }
  }

  if (isClosing) {
      Serial.print("Closing Encoder Count :");
  Serial.println(encoderCount);
    currentSpeed = max(30, calculateSpeed());
    Serial.print("Moving at Speed : ");
    Serial.println(currentSpeed);
    moveMotor(LOW);
    if (abs(encoderCount) >= maxPosition) {
      stopMotor();
      isClosing = false;
      Serial.println("🔒 Door Fully Closed.");
    }
  }
}
