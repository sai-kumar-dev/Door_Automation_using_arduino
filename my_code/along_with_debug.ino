#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8
#define OPEN_BUTTON 5
#define CLOSE_BUTTON 6 
#define home 2

volatile int encoderCount = 0;
const int maxPosition = 1000;
const int baseCloseDelay = 5000;

bool reached = false;
bool isOpening = false;
bool isClosing = false;
unsigned long closeStartTime = 0;
unsigned long closeDelay = baseCloseDelay;
int currentSpeed = 0;

// Debug state tracking to prevent repeated Serial messages
bool prevOpening = false;
bool prevClosing = false;
bool prevHomed = false;
bool prevStopped = true;

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
  pinMode(home, INPUT_PULLUP);
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.println("System Booting... Homing Sequence Started.");
  
  // Ensure motor moves indefinitely until homing is complete
  while (digitalRead(home) == HIGH) {
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 80);
  }
  stopMotor();
  reached = true;  // Homing complete
  Serial.println("Homing Complete. System Ready.");
}

void loop() {
  if (!reached) return;  // Prevent logic from running until homing is done

  // OPEN BUTTON LOGIC
  if (digitalRead(OPEN_BUTTON) == LOW) {
    if (isClosing) {
      // Emergency stop when closing
      stopMotor();
      isClosing = false;
      isOpening = false;
      Serial.println("!!! Emergency Stop Triggered. !!!");
    } else {
      if (!isOpening) {
        isOpening = true;
        Serial.println("[ACTION] Opening Door...");
      } else {
        closeDelay += 5000;
        Serial.print("[INFO] Close delay extended to: ");
        Serial.print(closeDelay / 1000);
        Serial.println(" sec.");
      }
    }
  }

  // CLOSE BUTTON LOGIC
  if (digitalRead(CLOSE_BUTTON) == LOW) {
    if (!isClosing) {
      isClosing = true;
      Serial.println("[ACTION] Closing Door...");
    } // No message needed if already closing
  }

  // OPENING PROCESS
  if (isOpening) {
    if (!prevOpening) {
      Serial.println("[STATUS] Door is Opening...");
      prevOpening = true;
      prevClosing = false;
      prevStopped = false;
    }
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(HIGH);
    if (abs(encoderCount) >= maxPosition) {
      stopMotor();
      isOpening = false;
      closeStartTime = millis();
      Serial.println("[STATUS] Door Fully Open. Auto-close Timer Started.");
    }
  }

  // AUTO-CLOSING AFTER DELAY
  if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
    isClosing = true;
    Serial.println("[AUTO] Door Closing Automatically...");
  }

  // CLOSING PROCESS
  if (isClosing) {
    if (!prevClosing) {
      Serial.println("[STATUS] Door is Closing...");
      prevClosing = true;
      prevOpening = false;
      prevStopped = false;
    }
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(LOW);
    if (encoderCount <= 0) {
      stopMotor();
      isClosing = false;
      closeDelay = baseCloseDelay; // Reset delay after full close
      Serial.println("[STATUS] Door Fully Closed. System Reset.");
    }
  }

  // STOPPED STATE (ENSURES NO REPEATED LOGGING)
  if (!isOpening && !isClosing && !prevStopped) {
    Serial.println("[IDLE] System is Waiting...");
    prevStopped = true;
    prevOpening = false;
    prevClosing = false;
  }
}
