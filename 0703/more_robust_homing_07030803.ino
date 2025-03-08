#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6 
#define HOME_SENSOR 2
#define EMERGENCY_STOP 7  // Emergency stop button

volatile int encoderCount = 0;
const int maxPosition = 1000;
const int baseCloseDelay = 5000;
const int homingTimeout = 10000;  // 10 seconds timeout for homing

bool reached = false;
bool isOpening = false;
bool isClosing = false;
bool isHoming = true;  // Start with homing
unsigned long closeStartTime = 0;
unsigned long closeDelay = baseCloseDelay;
unsigned long homingStartTime = 0;  // Track homing start time

int currentSpeed = 0;

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
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.println("System Booting... Homing in progress.");
  homingStartTime = millis();  // Start tracking homing time
}

void loop() {
  // HOMING PHASE
  if (isHoming) {
    if (digitalRead(EMERGENCY_STOP) == LOW) {  // Emergency Stop Pressed
      stopMotor();
      Serial.println("⚠️ Emergency Stop Triggered! Homing Halted.");
      isHoming = false;
      return;
    }

    if (digitalRead(HOME_SENSOR) == HIGH) {
      digitalWrite(DIR_PIN, HIGH);
      analogWrite(PWM_PIN, 80);
      Serial.println("🔄 Homing in Progress...");
    } else {
      stopMotor();
      isHoming = false;
      reached = true;
      Serial.println("✅ Homing Complete! System Ready.");
    }

    // Homing timeout fail-safe
    if (millis() - homingStartTime > homingTimeout) {
      stopMotor();
      Serial.println("⏳ Homing Timeout! Check Home Sensor.");
      isHoming = false;
    }
    return;  // Stop loop execution if still homing
  }

  // MAIN DOOR CONTROL LOGIC (After Homing)
  if (reached) {
    if (digitalRead(OPEN_BUTTON) == LOW) {
      if (isClosing) {
        isClosing = false;
        isOpening = true;
        Serial.println("🔄 Emergency Opening... (Interrupting Close)");
      } else {
        if (!isOpening) {
          isOpening = true;
          Serial.println("🚪 Opening...");
        } else {
          closeDelay += 5000;
          Serial.print("⏳ Extending Auto-Close Delay: ");
          Serial.println(closeDelay);
        }
      }
    }

    if (digitalRead(CLOSE_BUTTON) == LOW) {
      isClosing = true;
      Serial.println("🔒 Close Button Pressed.");
    }

    if (isOpening) {
      int speed = calculateSpeed(abs(encoderCount));
      moveMotor(HIGH);
      if (abs(encoderCount) >= maxPosition) {
        stopMotor();
        isOpening = false;
        closeStartTime = millis();
        Serial.println("✅ Door Fully Open.");
      }
      Serial.print("📊 Opening Position: ");
      Serial.println(encoderCount);
    }

    if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
      isClosing = true;
      Serial.println("⏳ Auto-Closing...");
    }

    if (isClosing) {
      int speed = calculateSpeed(abs(encoderCount));
      moveMotor(LOW);
      if (encoderCount <= 0) {
        stopMotor();
        isClosing = false;
        closeDelay = baseCloseDelay;  // Reset delay after full close
        Serial.println("🔒 Door Fully Closed.");
      }
      Serial.print("📊 Closing Position: ");
      Serial.println(encoderCount);
    }
  }
}
