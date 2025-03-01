// Door Automation with Quadrature Encoder

const int dirPin = 8;
const int pwmPin = 9;

const int encoderAPin = 2; // Interrupt pin
const int encoderBPin = 3;

volatile long encoderCount = 0; // Volatile because it's modified in an interrupt

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin), updateEncoder, CHANGE);
  Serial.begin(9600);
}

void loop() {
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
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
