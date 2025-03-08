#define ENCODER_A 2  // Encoder A pin
#define ENCODER_B 3  // Encoder B pin
#define PWM_PIN 9    // Motor PWM control
#define DIR_PIN 8    // Motor direction control

volatile int encoderCount = 0;
int targetPosition = 0;
bool isMoving = false;

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
    isMoving = false;
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
        String input = Serial.readStringUntil('\n');
        input.trim();
        char direction = input.charAt(0);
        int distance = input.substring(1).toInt();
        
        if (direction == 'f') {
            targetPosition = encoderCount + distance;
            isMoving = true;
            moveMotor(HIGH, 150);
        } else if (direction == 'b') {
            targetPosition = encoderCount - distance;
            isMoving = true;
            moveMotor(LOW, 150);
        }
    }
    
    if (isMoving) {
        if ((encoderCount >= targetPosition && targetPosition > encoderCount) ||
            (encoderCount <= targetPosition && targetPosition < encoderCount)) {
            stopMotor();
        }
    }
}
