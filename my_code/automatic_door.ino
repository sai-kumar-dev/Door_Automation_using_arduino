#define ENCODER_A 2  // Encoder A pin
#define ENCODER_B 3  // Encoder B pin
#define PWM_PIN 9    // Motor PWM control
#define DIR_PIN 8    // Motor direction control

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
}

int calculateSpeed(int position) {
    int minSpeed = 50;
    int maxSpeed = 150;
    int range = maxPosition / 3; // Define slow-down range near edges
    if (position < range || position > (maxPosition - range)) {
        return minSpeed + ((maxSpeed - minSpeed) * position) / range;
    }
    return maxSpeed;
}

void setup() {
    Serial.begin(9600);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
}

void loop() {
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'o') {
            if (isClosing) {
                for (int speed = 150; speed >= 50; speed -= 10) {
                    moveMotor(LOW, speed);
                    delay(50);
                }
                stopMotor();
            }
            isOpening = true;
            isClosing = false;
            closeStartTime = millis(); // Extend delay when 'o' is pressed again
        } else if (command == 'c' && !isOpening) { // Ignore closing command if opening
            isClosing = true;
            closeStartTime = millis();
        }
    }
    
    if (isOpening) {
        int speed = calculateSpeed(encoderCount);
        moveMotor(HIGH, speed);
        if (encoderCount >= maxPosition) {
            stopMotor();
            isOpening = false;
            closeStartTime = millis();
        }
    }
    
    if (!isOpening && isClosing && millis() - closeStartTime > closeDelay) {
        int speed = calculateSpeed(encoderCount);
        moveMotor(LOW, speed);
        if (encoderCount <= 0) {
            for (int speed = 50; speed <= 150; speed += 10) {
                moveMotor(LOW, speed);
                delay(50);
            }
            stopMotor();
            isClosing = false;
        }
    }
}
