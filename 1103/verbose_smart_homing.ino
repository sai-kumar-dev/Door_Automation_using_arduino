// =============================
// Smart Glass Door Controller
// Verbose Mode & Smooth Speed with Intelligent Homing
// =============================

#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6
#define HOME 11

#define VERBOSE_MODE true  // Set to false to reduce Serial output

volatile int encoderCount = 0;
const int maxPosition = 1050;
const int minSpeed = 60;
const int maxSpeed = 100;

bool reachedHome = false;
bool isOpening = false;
bool isClosing = false;

int currentSpeed = 0;
bool prevOpening = false;
bool prevClosing = false;
bool prevStopped = true;
bool prevHomed = false;

unsigned long lastEncoderChangeTime = 0;
int lastEncoderValue = 0;

void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void verbosePrint(String msg) {
  if (VERBOSE_MODE) {
    Serial.println(msg);
  }
}

int getSmoothSpeed(int currentPos, int targetPos) {
  int distance = abs(targetPos - currentPos);
  int dynamicSpeed = map(distance, 0, maxPosition, minSpeed, maxSpeed);
  dynamicSpeed = constrain(dynamicSpeed, minSpeed, maxSpeed);
  return dynamicSpeed;
}

void moveMotor(int direction, int speedVal) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, speedVal);
  currentSpeed = speedVal;
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  currentSpeed = 0;
  verbosePrint("[ACTION] Motor stopped.");
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

  verbosePrint("[BOOT] System initialized. Waiting for homing...");
  delay(1000);
}

void loop() {
  // ================= HOMING ===================
  if (!reachedHome) {
    static int homingSpeed = minSpeed;
    static unsigned long lastAdjustTime = 0;
    unsigned long currentTime = millis();

    // Check if door is moving (encoder count changes)
    if (currentTime - lastAdjustTime > 300) {
      if (encoderCount == lastEncoderValue) {
        homingSpeed = min(homingSpeed + 5, maxSpeed);
        verbosePrint("[HOMING] Increasing speed to overcome inertia: " + String(homingSpeed));
      } else {
        homingSpeed = max(homingSpeed - 5, minSpeed);
        verbosePrint("[HOMING] Reducing speed for accuracy: " + String(homingSpeed));
      }
      lastAdjustTime = currentTime;
      lastEncoderValue = encoderCount;
    }

    if (digitalRead(HOME) == HIGH) {
      moveMotor(HIGH, homingSpeed);
      if (!prevHomed) {
        Serial.println("🔄 Homing in progress...");
        prevHomed = true;
      }
    } else {
      stopMotor();
      encoderCount = 0;
      reachedHome = true;
      Serial.println("✅ Homing complete. Door is fully opened.");
    }
    return;
  }

  // ============= BUTTON HANDLING =============
  bool openPressed = digitalRead(OPEN_BUTTON) == LOW;
  bool closePressed = digitalRead(CLOSE_BUTTON) == LOW;

  if (openPressed && !isOpening) {
    if (encoderCount <= 0) {
      Serial.println("⚠️ Door already fully open.");
    } else {
      isOpening = true;
      isClosing = false;
      Serial.println("🔼 Opening door...");
    }
  }

  if (closePressed && !isClosing) {
    if (encoderCount >= maxPosition) {
      Serial.println("⚠️ Door already fully closed.");
    } else {
      isClosing = true;
      isOpening = false;
      Serial.println("🔽 Closing door...");
    }
  }

  if (isOpening) {
    if (!prevOpening) {
      verbosePrint("[STATUS] Opening...");
      prevOpening = true;
      prevClosing = false;
      prevStopped = false;
    }
    int speed = getSmoothSpeed(encoderCount, 0);
    moveMotor(HIGH, speed);
    if (encoderCount <= 0) {
      stopMotor();
      isOpening = false;
      Serial.println("✅ Door fully opened.");
    }
    return;
  }

  if (isClosing) {
    if (!prevClosing) {
      verbosePrint("[STATUS] Closing...");
      prevClosing = true;
      prevOpening = false;
      prevStopped = false;
    }
    int speed = getSmoothSpeed(encoderCount, maxPosition);
    moveMotor(LOW, speed);
    if (encoderCount >= maxPosition) {
      stopMotor();
      isClosing = false;
      Serial.println("🔒 Door fully closed.");
    }
    return;
  }

  if (!isOpening && !isClosing && !prevStopped) {
    Serial.println("[IDLE] Waiting for input...");
    prevStopped = true;
    prevOpening = false;
    prevClosing = false;
  }
}
