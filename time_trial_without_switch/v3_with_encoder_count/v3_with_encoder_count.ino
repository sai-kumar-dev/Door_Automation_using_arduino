const int dirPin = 8;
const int pwmPin = 9;

const int encoderAPin = 2; // Interrupt pin
const int encoderBPin = 3;
unsigned long openDuration = 1000; // Milliseconds for opening (adjust)
unsigned long closeDuration = 1000; // Milliseconds for closing (adjust)
unsigned long timeoutDuration = 1000; // Timeout (adjust)
unsigned long extra = 0;
volatile long encoderCount = 0;
float prevcount = 0;

const int maxSpeed = 255;
const int minSpeed = 80;

unsigned long lastOpenTime = 0;
bool doorOpen = false;
bool isClosing = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin), handleencoder, RISING );
  Serial.begin(9600);
  Serial.println("Enter 'o' to open, 'c' to close.");
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
}

void handleencoder() {
  if (digitalRead(encoderAPin) > digitalRead(encoderBPin))
  { encoderCount++;}
  else {encoderCount--;}
  
  prevcount=encoderCount;
}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'o') {
        Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
      lastOpenTime = currentTime;
      if (!doorOpen) {
        openDoor();
        doorOpen = true;
        isClosing = false;
      } else if (isClosing) {
        openDoor();
        isClosing = false;
        doorOpen = true;
      }
    } else if (command == 'c') {
      if (doorOpen) {
        closeDoor();
        doorOpen = false;
        isClosing = true;
      }
    }
  }

  if (doorOpen && (currentTime - lastOpenTime >= openDuration)) {
    closeDoor();
    doorOpen = false;
    isClosing = true;
  }

  if (doorOpen && (currentTime - lastOpenTime >= timeoutDuration)) {
    Serial.println("Timeout: Closing door.");
    closeDoor();
    doorOpen = false;
    isClosing = true;
  }
}

void openDoor() {
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
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
  Serial.println("Now waiting for timeout Duration ......");
  delay(timeoutDuration + extra);
//closeDoor();
}

void closeDoor() {
  digitalWrite(dirPin, LOW); // Close direction
  lastOpenTime = millis();
  Serial.println("Closing door...");
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
  for (unsigned long elapsedTime = 0; elapsedTime < closeDuration; elapsedTime += 20) {
    int currentSpeed = calculateSpeed(elapsedTime, closeDuration);
    analogWrite(pwmPin, currentSpeed);
    if (Serial.available() > 0 && Serial.read() == 'o') {
      stopMotor();
      Serial.println("opening during closing due to open signal ......");
      Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
      openDoor();
      isClosing = false;
      doorOpen = true;
      return;
    }
    delay(20);
  }
  stopMotor();
  isClosing = false;
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
}

void stopMotor() {
  analogWrite(pwmPin, 0);
  Serial.println("Motor stopped.");
}

int calculateSpeed(unsigned long elapsedTime, unsigned long totalDuration) {
  float progress = (float)elapsedTime / totalDuration;
  int speed = minSpeed + (int)(progress * (maxSpeed - minSpeed));
  if (elapsedTime > (totalDuration / 2)) {
    speed = maxSpeed - (int)((progress) * (maxSpeed - minSpeed));
  }
  return constrain(speed, minSpeed, maxSpeed);
}

void updateEncoder() {if (digitalRead(encoderAPin) > digitalRead(encoderBPin))
  { encoderCount++;}
  else {encoderCount--;}
  prevcount = encoderCount;
}
