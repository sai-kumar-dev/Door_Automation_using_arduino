#define ENCODER_A 3
#define ENCODER_B 4
#define PWM_PIN 9
#define DIR_PIN 8

volatile int encoderCount = 0;
const int maxPosition = 1000;
const int closeDelay = 5000;  

bool isOpening = false;
bool isClosing = false;
unsigned long closeStartTime = 0;
unsigned long lastEncoderTime = 0;

int currentSpeed = 0;
int lastEncoderValue = 0;  // For detecting stuck movement

void encoderISR() {
  // Improved encoder direction detection
  if (digitalRead(ENCODER_A) ^ digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
  lastEncoderTime = millis();  // Store last encoder movement time
}

// Function for smooth speed variation using millis()
int calculateSpeed(int position) {
  int minSpeed = 80;
  int maxSpeed = 180;
  int slowZone = maxPosition * 0.2;
  int mediumZoneEnd = maxPosition * 0.8;
  int speedStep = 2;  // Smaller steps for smoother transitions
  unsigned long speedUpdateInterval = 50; // Update speed every 50ms
  static unsigned long lastSpeedUpdate = 0;
  
  int targetSpeed = (position < slowZone || position > mediumZoneEnd) ? minSpeed : maxSpeed;

  if (millis() - lastSpeedUpdate > speedUpdateInterval) {
    lastSpeedUpdate = millis();
    if (currentSpeed < targetSpeed) {
      currentSpeed = min(currentSpeed + speedStep, targetSpeed);
    } else if (currentSpeed > targetSpeed) {
      currentSpeed = max(currentSpeed - speedStep, targetSpeed);
    }
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
}

void loop() {
  // Serial command handling
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'o') {
      if (isClosing) {
        stopMotor();
        isClosing = false;
      }
      isOpening = true;
      closeStartTime = millis();
      Serial.println("Opening...");
    } else if (command == 'c' && !isOpening) {
      isClosing = true;
      closeStartTime = millis();
      Serial.println("Closing...");
    }
  }

  // Check if encoder is stuck
  if (millis() - lastEncoderTime > 2000) {  // No movement detected for 2 sec
    Serial.println("⚠️ Encoder Stuck - Stopping Motor!");
    stopMotor();
    isOpening = false;
    isClosing = false;
  }

  // Opening Logic
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

  // Closing Logic
  if (!isOpening && isClosing && millis() - closeStartTime > closeDelay) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(LOW);
    if (encoderCount >= 0) {
      stopMotor();
      isClosing = false;
      Serial.println("Door Fully Closed");
    }
    Serial.print("Closing, Encoder: ");
    Serial.println(encoderCount);
  }
}
