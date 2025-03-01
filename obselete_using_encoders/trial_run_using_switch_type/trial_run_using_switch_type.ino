// Door Automation with Encoder-Based Positioning and Speed Control

const int dirPin = 8;
const int pwmPin = 9;

const int encoderA = 2; // Interrupt pin
const int encoderB = 3;

volatile long encoderCount = 0;

long openEncoderCount = 5; // Replace with your actual open count
long closeEncoderCount = 0; // Replace with your actual close count

bool doorOpen = false;
bool isClosing = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
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
      if (!doorOpen || isClosing) {
        openDoor();
        doorOpen = true;
        isClosing = false;
      }
    }else if(command == 'c'){
      Serial.println("Closing ......");
      if(doorOpen || !isClosing){
      isClosing = true;
      closeDoor();
      }      
    }else{
      Serial.println("nothing doing");
    }
  }

  if (doorOpen) {
    if (encoderCount >= openEncoderCount) {
      stopMotor();
      doorOpen = false;
    } else {
      adjustSpeed(); // Adjust speed while opening
    }
  } else if (isClosing) {
    if (encoderCount <= closeEncoderCount - 2) {
      stopMotor();
      isClosing = false;
    } else {
      adjustSpeed(); // Adjust speed while closing
    }
  }

  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
}

void openDoor() {
  digitalWrite(dirPin, LOW); // Open direction
  analogWrite(pwmPin, 200); // Start motor
  doorOpen = true;
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
  if (digitalRead(encoderA) > digitalRead(encoderB))
  { encoderCount++;}
  else {encoderCount--;}
  
  prevenccount=encoderCount;
}

void adjustSpeed() {
  // Simplified speed control based on encoder count
  long targetCount = doorOpen ? openEncoderCount : closeEncoderCount;
  long distanceToTarget = abs(encoderCount - targetCount);

  int speed = 200; // Default speed
  if (distanceToTarget < 200) {
    speed = 150; // Slow down near target
  }
  if (distanceToTarget < 100) {
    speed = 100;
  }
  if (distanceToTarget < 50){
    speed = 80;
  }
  analogWrite(pwmPin, speed);
}
