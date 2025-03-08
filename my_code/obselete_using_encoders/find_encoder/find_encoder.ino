    const int encoderPinA = 2; // Replace with your pin number
    const int encoderPinB = 3; // Replace with your pin number

    void setup() {
      pinMode(encoderPinA, INPUT_PULLUP);
      pinMode(encoderPinB, INPUT_PULLUP);
      Serial.begin(9600);
    }

    void loop() {
      int stateA = digitalRead(encoderPinA);
      int stateB = digitalRead(encoderPinB);
      Serial.print("A: ");
      Serial.print(stateA);
      Serial.print(" B: ");
      Serial.println(stateB);
      delay(10); // Adjust delay as needed
    }
