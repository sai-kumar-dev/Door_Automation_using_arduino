// Door Automation with Encoder-Based Positioning and Speed Control

const int dirPin = 8;
const int pwmPin = 9;
const int maxSpeed = 255;
const int minSpeed = 80;
const int encoderAPin = 2; // Interrupt pin
const int encoderBPin = 3;
unsigned long lastOpenTime = 0;
unsigned long openDuration = 2000; // Milliseconds for opening (adjust)
unsigned long closeDuration = 2000; // Milliseconds for closing (adjust)
unsigned long timeoutDuration = 10000; // Timeout (adjust)
unsigned long extra = 0;

volatile long encoderCount = 0;

long openEncoderCount = 5; // Replace with your actual open count
long closeEncoderCount = 0; // Replace with your actual close count

bool doorOpen = false;
bool isClosing = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), updateEncoder, CHANGE);
  Serial.begin(9600);
  Serial.println("Enter 'o' to simulate motion.");
}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'o') {
      
      Serial.println("Opening .......");
      Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
      if (!doorOpen || isClosing) {
        openDoor();
        doorOpen = true;
        isClosing = false;
      }
    }
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
}
}

void openDoor() {
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);

  extra = 0;
  digitalWrite(dirPin, HIGH); // Open direction
  lastOpenTime = millis();
  Serial.println("Opening door...");
  for (unsigned long elapsedTime = 0; elapsedTime < openDuration; elapsedTime += 20) {
    int currentSpeed = calculateSpeed(elapsedTime, openDuration);
    analogWrite(pwmPin, currentSpeed);
    delay(20);
    if (Serial.available() > 0 && Serial.read() == 'o') {
      stopMotor();
      Serial.println("Opening due to open signal even during open....");
      extra = 5000;
    }
  }
  stopMotor();
  Serial.println("Now waiting for timeout Duration ......");
  delay(timeoutDuration + extra);
//closeDoor();
Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
}

void closeDoor() {
  digitalWrite(dirPin, HIGH); // Close direction
  analogWrite(pwmPin, 200); // Start motor
  isClosing = true;
}

void stopMotor() {
  analogWrite(pwmPin, 0); // Stop motor
}

void updateEncoder() {
  int a = digitalRead(encoderAPin);
  int b = digitalRead(encoderBPin);

  if (a == HIGH) {
    if (b == LOW) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  } else {
    if (b == HIGH) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
}

int calculateSpeed(unsigned long elapsedTime, unsigned long totalDuration) {
  float progress = (float)elapsedTime / totalDuration;
  int speed = minSpeed + (int)(progress * (maxSpeed - minSpeed));
  if (elapsedTime > (totalDuration / 2)) {
    speed = maxSpeed - (int)((progress) * (maxSpeed - minSpeed));
  }
  return constrain(speed, minSpeed, maxSpeed);
}
