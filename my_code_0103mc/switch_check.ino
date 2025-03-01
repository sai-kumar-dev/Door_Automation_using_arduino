#define ENCODER_A 2  // Encoder A pin
#define ENCODER_B 3  // Encoder B pin
#define PWM_PIN 9    // Motor PWM control
#define DIR_PIN 8    // Motor direction control
#define OPEN_BUTTON 4 // Open button pin
#define CLOSE_BUTTON 5 // Close button pin

volatile int encoderCount = 0;
const int maxPosition = 1000; // Adjust based on actual door travel
const int closeDelay = 5000;  // 5 seconds delay before closing
bool isOpening = false;
bool isClosing = false;
unsigned long closeStartTime = 0;

void IRAM_ATTR encoderISR() {
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void moveMotor(int direction, int speed) {
    digitalWrite(DIR_PIN, direction);
    analogWrite(PWM_PIN, speed);
}

void stopMotor() {
    analogWrite(PWM_PIN, 0);
    isOpening = false;
    isClosing = false;
}

void setup() {
    Serial.begin(9600);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(OPEN_BUTTON, INPUT_PULLUP);
    pinMode(CLOSE_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
}

void loop() {
    if (digitalRead(OPEN_BUTTON) == LOW) {
        if (isClosing) {
            stopMotor(); // Stop closing before opening
        }
        isOpening = true;
        isClosing = false;
        moveMotor(HIGH, 150);
    }
    
    if (digitalRead(CLOSE_BUTTON) == LOW) {
        if (isOpening) {
            stopMotor(); // Stop opening before closing
        }
        isOpening = false;
        isClosing = true;
        closeStartTime = millis();
    }
    
    if (isOpening && encoderCount >= maxPosition) {
        stopMotor();
        closeStartTime = millis(); // Set delay before closing
    }
    
    if (!isOpening && isClosing && millis() - closeStartTime > closeDelay) {
        moveMotor(LOW, 150);
    }
    
    if (isClosing && encoderCount <= 0) {
        stopMotor();
    }
}
