#define ENCODER_A 3   // Encoder A pin (connected to encoder's A output)
#define ENCODER_B 4   // Encoder B pin (connected to encoder's B output)
#define PWM_PIN 9     // Motor PWM control pin (connected to motor driver's PWM input)
#define DIR_PIN 8     // Motor direction control pin (connected to motor driver's direction input)
float prev = 0;
volatile int encoderCount = 0; // Variable to store the encoder count (volatile because it's modified in an interrupt)
const int maxPosition = 1000; // Maximum encoder count representing the fully open position (adjust based on your door)
const int closeDelay = 1000;  // Delay (in milliseconds) before the door automatically closes (5 seconds)
bool isOpening = false;      // Flag indicating if the door is currently opening
bool isClosing = false;      // Flag indicating if the door is currently closing
unsigned long closeStartTime = 0; // Timestamp when the door should start closing

// Interrupt Service Routine (ISR) for encoder pulses
void encoderISR() {
  // Quadrature decoding: determine rotation direction based on A and B pin states
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++; // Increment count if A and B are the same
  } else {
    encoderCount--; // Decrement count if A and B are different
  }
  prev = encoderCount;
}

// Function to move the motor in a specified direction and speed
void moveMotor(int direction, int speed) {
  digitalWrite(DIR_PIN, direction); // Set motor direction
  analogWrite(PWM_PIN, speed);     // Set motor speed using PWM
}

// Function to stop the motor
void stopMotor() {
  analogWrite(PWM_PIN, 0); // Set PWM to 0 to stop the motor
}

// Function to calculate motor speed based on the door's position
int calculateSpeed(int position) {
  int minSpeed = 80;  // Minimum speed (slow speed)
  int mediumSpeed = 180; // Medium speed
  int slowZone = maxPosition * 0.2; // 20% of the total travel for slow zones
  int mediumZoneStart = slowZone;
  int mediumZoneEnd = maxPosition * 0.8; // 80% of the total travel

  // Determine speed based on position
  if (position < slowZone || position > mediumZoneEnd) {
    return minSpeed; // Slow speed in the slow zones
  } else if (position >= mediumZoneStart && position <= mediumZoneEnd) {
    return mediumSpeed; // Medium speed in the middle zone
  } else {
    return minSpeed; // Default to slow in case of unexpected position
  }
}

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING); // Attach interrupt to encoder A pin, trigger on any change
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'o') { // Open command
      if (isClosing) { // If closing, stop it
        stopMotor();
        isClosing = false;
      }
      isOpening = true; // Set opening flag
      closeStartTime = millis(); // Reset close delay
      Serial.println("Opening command received"); // Debug
    } else if (command == 'c' && !isOpening) { // Close command (ignore if opening)
      isClosing = true; // Set closing flag
      closeStartTime = millis(); // Set close start time
      Serial.println("Closing command received"); // Debug
    }
  }

  // Opening logic
  if (isOpening) {
    int speed = calculateSpeed(encoderCount); // Calculate speed based on position
    moveMotor(HIGH, speed); // Move motor to open
    if (abs(encoderCount) >= maxPosition) { // If reached max position, stop
      stopMotor();
      isOpening = false;
      closeStartTime = millis(); // Reset close delay
      Serial.println("Door fully open"); // Debug
    }
    Serial.print("Opening, Encoder: "); // Debug
    Serial.println(encoderCount);
  }

  // Closing logic
  if (!isOpening && isClosing && millis() - closeStartTime > closeDelay) {
    //int speed = calculateSpeed(abs(encoderCount)); // Calculate speed based on position
    moveMotor(LOW, 180); // Move motor to close
    if (encoderCount >= 0) { // If reached closed position, stop
      stopMotor();
      isClosing = false;
      Serial.println("Door fully closed"); // Debug
    }
    Serial.print("Closing, Encoder: "); // Debug
    Serial.println(encoderCount);
  }

  // Debugging Serial output (uncomment to use)
  // Serial.print("Encoder: ");
  // Serial.print(encoderCount);
  // Serial.print(", Opening: ");
  // Serial.print(isOpening);
  // Serial.print(", Closing: ");
  // Serial.print(isClosing);
  // Serial.print(", Speed: ");
  // Serial.println(calculateSpeed(encoderCount));
}

/*
Example Expected Working:

1. Upload the code to your Arduino.
2. Open the Serial Monitor at 9600 baud.
3. Send 'o' followed by Enter.
   - The door should start opening.
   - The serial monitor should print "Opening command received" and then print "Opening, Encoder: [current encoder count]" repeatedly.
   - The door should slow down as it approaches the fully open position.
   - Once the door reaches the fully open position, the serial monitor should print "Door fully open".
4. After 5 seconds (closeDelay), the door should start closing automatically.
   - The serial monitor should print "Closing, Encoder: [current encoder count]" repeatedly.
   - The door should slow down as it approaches the fully closed position.
   - Once the door reaches the fully closed position, the serial monitor should print "Door fully closed".
5. Send 'o' followed by Enter while the door is closing.
   - The door should stop closing immediately.
   - The door should start opening.
   - The automatic close delay should reset.
6. Send 'c' followed by Enter.
   - The door should start closing.
   - The automatic close delay should start.
7. If the encoder counts are not increasing or decreasing as expected, check your encoder wiring.
8. If the motor is moving in the wrong direction, swap the motor wires or change the HIGH and LOW values in the moveMotor() function.
9. Adjust maxPosition, minSpeed, mediumSpeed, and closeDelay to fit your specific setup.
*/
