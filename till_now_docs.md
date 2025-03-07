**Arduino-Based Automatic Door Control System Documentation**

### **Overview**
The Arduino-based automatic door control system is designed to control the opening and closing of a door using a single "Open" button and an additional "Close" button for manual override. The system utilizes an encoder to track the position of the door and ensures smooth movement through a speed variation algorithm. The door automatically closes after a delay, which increases if the "Open" button is pressed again during operation. If the door is closing and "Open" is pressed, it retracts and remains open until the delay expires.

---

### **Hardware Components**
1. **Arduino Board (e.g., Arduino Uno)** – Microcontroller for processing inputs and controlling the motor.
2. **Motor with Encoder** – Drives the door movement while tracking its position.
3. **Push Buttons**:
   - **Open Button** – Initiates door opening and increases delay if pressed again.
   - **Close Button** – Manually closes the door at any time.
4. **Power Supply (SMPS 5V)** – Provides power to the system.
5. **Motor Driver Circuit** – Controls the motor's speed and direction.
6. **Connecting Wires** – For circuit connections.

---

### **Pin Configuration**
| Component       | Pin  |
|----------------|------|
| Encoder A      | 3    |
| Encoder B      | 4    |
| PWM Output     | 9    |
| Motor Direction | 8    |
| Open Button    | 5    |
| Close Button   | 6    |

---

### **Software Logic**
The code is written in **C++ for Arduino** and includes the following key functionalities:

#### **1. Encoder Handling**
- The encoder keeps track of the motor's position.
- An interrupt is attached to **Encoder A** to update the count whenever the motor moves.

#### **2. Speed Control Algorithm**
- The motor's speed varies based on its position to ensure smooth acceleration and deceleration.
- Minimum and maximum speed values are set.
- The speed increases or decreases gradually to prevent jerky movements.

#### **3. Button-Based Operation**
- **Open Button:**
  - Pressing once starts opening.
  - Pressing again while opening increases the auto-close delay by **5 seconds**.
  - Pressing during closing retracts the door and keeps it open.
- **Close Button:**
  - Manually closes the door at any point.
  
#### **4. Automatic Closing Mechanism**
- Once fully opened, the system waits for the delay period.
- If no input is received, the door automatically closes.
- The delay resets after a full close.

---

### **Code Walkthrough**
```cpp
#define ENCODER_A 3  // Encoder A pin
#define ENCODER_B 4  // Encoder B pin
#define PWM_PIN 9    // PWM pin for motor speed control
#define DIR_PIN 8    // Direction pin for motor
#define OPEN_BUTTON 5 // Open button pin
#define CLOSE_BUTTON 6 // Close button pin
```
- **Defines pin connections for the motor, encoder, and buttons.**

```cpp
volatile int encoderCount = 0;
const int maxPosition = 1000;
const int baseCloseDelay = 5000;
```
- **`encoderCount`** tracks the door’s position.
- **`maxPosition`** sets the fully open position.
- **`baseCloseDelay`** is the default auto-close delay (5 seconds).

```cpp
void encoderISR() {
  if (digitalRead(ENCODER_A) > digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
```
- **Interrupt function updates the encoder count based on movement direction.**

```cpp
int calculateSpeed(int position) {
  int minSpeed = 80;
  int maxSpeed = 180;
  int slowZone = maxPosition * 0.2;
  int mediumZoneEnd = maxPosition * 0.8;
  int targetSpeed = (position < slowZone || position > mediumZoneEnd) ? minSpeed : maxSpeed;
  return targetSpeed;
}
```
- **Determines motor speed based on door position for smooth motion.**

```cpp
void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OPEN_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_BUTTON, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
}
```
- **Initializes pins and sets up the encoder interrupt.**

```cpp
void loop() {
  if (digitalRead(OPEN_BUTTON) == HIGH) {
    if (isClosing) {
      isClosing = false;
      isOpening = true;
      Serial.println("Retracting - Opening again...");
    } else {
      if (!isOpening) {
        isOpening = true;
        Serial.println("Opening...");
      } else {
        closeDelay += 5000;
        Serial.print("Extending delay: ");
        Serial.println(closeDelay);
      }
    }
  }
}
```
- **Handles Open button logic:**
  - Starts opening if pressed.
  - Increases delay if pressed again.
  - Retracts if pressed during closing.

```cpp
void loop() {
  if (digitalRead(CLOSE_BUTTON) == HIGH) {
    if (isOpening) {
      isOpening = false;
      isClosing = true;
      Serial.println("Manual Close Triggered...");
    } else if (!isClosing) {
      isClosing = true;
      Serial.println("Closing...");
    }
  }
}
```
- **Handles Close button logic for manual control.**

```cpp
if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
  isClosing = true;
  Serial.println("Closing...");
}
```
- **Automatically closes the door if no input is received after the delay.**

```cpp
void moveMotor(int direction) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, currentSpeed);
}
```
- **Controls the motor's direction and speed.**

---

### **Conclusion**
This automatic door system provides a smart, smooth, and user-friendly operation. The Open button allows intuitive delay extension, and the Close button ensures safety by allowing manual control. The encoder-based tracking ensures precision in movement, and the gradual speed variation prevents jerky motions.

---

### **Future Improvements**
1. **Infrared or Ultrasonic Sensor** – Detects obstructions and prevents the door from closing on an object.
2. **Remote Control Integration** – Operate the door via a smartphone or remote device.
3. **Battery Backup** – Ensures functionality during power failure.

This system can be enhanced further to provide smart automation and safety features.

