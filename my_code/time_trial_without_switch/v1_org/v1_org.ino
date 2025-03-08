const int dirPin = 8;
const int pwmPin = 9;

unsigned long openDuration = 2000; // Milliseconds for opening (adjust)
unsigned long closeDuration = 2000; // Milliseconds for closing (adjust)
unsigned long timeoutDuration = 10000; // Timeout (adjust)

const int maxSpeed = 255;
const int minSpeed = 80;

unsigned long lastOpenTime = 0;
bool doorOpen = false;
bool isClosing = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter 'o' to open, 'c' to close.");
}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'o') {
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
      openDoor();
      return;
    }
  }
  stopMotor();
  Serial.println("Now waiting for timeout Duration ......");
  delay(timeoutDuration);
}

void closeDoor() {
  digitalWrite(dirPin, LOW); // Close direction
  lastOpenTime = millis();
  Serial.println("Closing door...");
  for (unsigned long elapsedTime = 0; elapsedTime < closeDuration; elapsedTime += 20) {
    int currentSpeed = calculateSpeed(elapsedTime, closeDuration);
    analogWrite(pwmPin, currentSpeed);
    if (Serial.available() > 0 && Serial.read() == 'o') {
      stopMotor();
      Serial.println("opening during closing due to open signal ......");
      openDoor();
      isClosing = false;
      doorOpen = true;
      return;
    }
    delay(20);
  }
  stopMotor();
  isClosing = false;
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
