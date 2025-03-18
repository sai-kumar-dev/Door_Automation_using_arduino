 // === Pin Definitions ===
#define ENCODER_A 3           // Encoder channel A (interrupt pin)
#define ENCODER_B 4           // Encoder channel B
#define HOME_SENSOR 11         // Home position sensor (active LOW with INPUT_PULLUP)
#define OPEN_BTN 5            // Open button (active LOW)
#define CLOSE_BTN 6           // Close button (active LOW)
#define DIR_PIN 8             // Motor direction pin
#define PWM_PIN 9           // Motor PWM speed control

// === Motor Speed Control Constants ===
#define MAX_SPEED 180         // Highest motor speed (PWM)
#define MIN_SPEED 40         // Minimum speed near end travel
#define SPEED_STEP 10         // Speed increment step (not used here, kept for reference)

// === Door Positioning Constants ===
#define MAX_POSITION -1000  // Encoder count when door is fully closed
#define OPEN_SLOWDOWN 0.4 * MAX_POSITION    // Encoder position to begin slowdown during opening
#define CLOSE_SLOWDOWN 0.6 * MAX_POSITION   // Encoder position to begin slowdown during closing

// === State Variables ===
volatile int encoderCount = 0;    // Tracks door position based on encoder
volatile int sp = 0;

bool isHomed = false;             // Flag: has the door been homed?
bool isOpening = false;           // Flag: is door currently opening?
bool isClosing = false;           // Flag: is door currently closing?


// === Setup Function ===
void setup() {
  // Configure I/O pins
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(HOME_SENSOR, INPUT_PULLUP);
  pinMode(OPEN_BTN, INPUT_PULLUP);
  pinMode(CLOSE_BTN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);

  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Door System Booted ===");
  Serial.println("Starting homing sequence...");
  homeDoor(); // Initiate homing on boot
}

// === Main Loop ===
void loop() {
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
  // Prevent any operation before homing
  if (!isHomed) return;

  if (digitalRead(OPEN_BTN) == LOW) {
    handleOpen();
  } else if (digitalRead(CLOSE_BTN) == LOW) {
    handleClose();
  }
  delay(100); // simple debounce
}

// === Encoder ISR ===
void updateEncoder() {
  // Quadrature decoding
  encoderCount += (digitalRead(ENCODER_B) < digitalRead(ENCODER_A)) ? -1 : 1;
}

// === Homing Logic ===
void homeDoor() {
  sp = 140;
  digitalWrite(DIR_PIN, HIGH);             // Set direction to open
  analogWrite(PWM_PIN, sp);       // Start at max speed
  Serial.println("Homing Moving towards home sensor...");

  while (digitalRead(HOME_SENSOR) == HIGH) {
    Serial.print("Homing Encoder: ");
    Serial.println(encoderCount);
    delay(50); 
    sp = max(MIN_SPEED, sp -= 30);// Polling interval
  }

  analogWrite(PWM_PIN, 0);     // Stop motor
  encoderCount = 0;              // Reset encoder at home
  isHomed = true;
  Serial.println("Homing Completed. Encoder reset to 0.");
  Serial.println("========================================");
  Serial.println("Ready for operation. Awaiting input...");
}

// === Handle Open Button Press ===
void handleOpen() {
    if (isOpening) {
      Serial.println("[INFO] Door is already opening.");
      return;
    }
  
    if (encoderCount >= 0) {
      Serial.println("[INFO] Door is already fully open.");
      return;
    }
  
    Serial.println("[ACTION] Opening door...");
    isOpening = true;
    isClosing = false;
  
    // Move to encoderCount = 0 (open)
    moveDoor(HIGH, -20);
    encoderCount = 0;
  }

// === Handle Close Button Press ===
void handleClose() {
  if (isClosing) {
    Serial.println("INFO Door is already closing.");
    return;
  }

  if (encoderCount <= MAX_POSITION) {
    Serial.println("INFO Door is already fully closed.");
    return;
  }

  Serial.println("ACTION Closing door...");
  isClosing = true;
  isOpening = false;

  // Move to encoderCount = MAX_POSITION (close)
  moveDoor(LOW, MAX_POSITION);
}

int dist(int d, int a, int b){
  if(d == 0){
    return abs(a);
  }else{
    return a - b;
  }
}

// === General Motor Movement with S-curve Logic ===
void moveDoor(bool direction,  int target) {
    digitalWrite(DIR_PIN, direction); // Set motor direction
    int speed = MAX_SPEED;
    analogWrite(PWM_PIN, speed);
  
    Serial.print("[MOVING] ");
    Serial.println(direction == HIGH ? "Opening..." : "Closing...");
  
    while ((direction == HIGH && encoderCount <= target ) || (direction == LOW && encoderCount >= target)) {
      // Calculate dynamic speed adjustment based on distance to slowdown point
      if (digitalRead(HOME_SENSOR) == LOW && direction == HIGH){
        break;
      }
      int distanceToSlowdown = dist(direction, encoderCount, target);
        speed = map(distanceToSlowdown, MAX_POSITION, 0, MAX_SPEED, MIN_SPEED);
        speed = constrain(speed, MIN_SPEED, MAX_SPEED);
     
  
      analogWrite(PWM_PIN, speed); // Apply dynamic speed
  
      // Print encoder and speed for debugging
      Serial.print("[ENC] ");
      Serial.print("Encoder: ");
      Serial.print(encoderCount);
      Serial.print(" | Speed: ");
      Serial.println(speed);
  
      // === Emergency Handling ===
      if (direction == HIGH && digitalRead(CLOSE_BTN) == LOW) {
        Serial.println("[EMERGENCY] Open interrupted by Close. Stopping...");
        emergencyStop();
        handleClose();
        return;
      }
  
      if (direction == LOW && (digitalRead(OPEN_BTN) == LOW )) {
        Serial.println("[EMERGENCY] Close interrupted by Open. Stopping...");
        emergencyStop();
        handleOpen();
        return;
      }
  
      delay(50); // Polling delay
    }
  
    analogWrite(PWM_PIN, 0); // Stop after target reached
    Serial.println("[DONE] Door movement complete.");
    isOpening = false;
    isClosing = false;
    Serial.println("========================================");
  }

// === Emergency Stop Function ===
void emergencyStop() {
  analogWrite(PWM_PIN, 0); // Cut motor power
  delay(200);                // Pause briefly
  Serial.println("INFO Motor stopped.");
}
