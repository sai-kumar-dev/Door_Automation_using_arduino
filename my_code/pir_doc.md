# **Arduino-Based Automatic Door Control System with PIR Sensor**

## **Introduction**
This project is an automatic door control system using an Arduino. The door operates based on a PIR sensor (motion detection) and manual button inputs. The system ensures safety, smooth movement, and adaptability based on real-time interactions.

## **Features**
✅ **PIR Sensor Integration** - Detects motion and opens the door automatically.  
✅ **Manual Open & Close Buttons** - Allows user override anytime.  
✅ **Smart Delay Mechanism** - Extends closing delay if motion is detected again.  
✅ **Smooth Speed Control** - Adjusts speed gradually for safe movement.  
✅ **Retract Feature** - If motion is detected during closing, the door reopens.  

## **Hardware Components**
- **Arduino Uno** (Microcontroller)
- **DC Motor with Encoder** (For door movement tracking)
- **Motor Driver Module** (Controls motor direction & speed)
- **PIR Sensor (HC-SR501)** (Detects human movement)
- **Push Buttons** (For manual Open/Close control)
- **Power Supply** (For motor and Arduino)

## **Pin Configuration**
| Component       | Pin Number |
|---------------|-----------|
| Encoder A     | 3         |
| Encoder B     | 4         |
| PWM Motor Control | 9    |
| Motor Direction | 8     |
| Open Button   | 5         |
| Close Button  | 6         |
| PIR Sensor (OUT) | 7    |

## **Working Principle**
1️⃣ **Motion Detected or Open Button Pressed** → The door starts opening.  
2️⃣ **Press Open Again or Detect Motion Again** → Adds 5 sec delay before closing.  
3️⃣ **No Motion for Delay Time** → Door Closes Automatically.  
4️⃣ **Motion Detected During Closing** → The door retracts (opens again).  
5️⃣ **Close Button Pressed** → Manually closes the door.

## **Code Explanation**
### **1. Initializing Pins and Variables**
```cpp
#define ENCODER_A 3  // Encoder A pin
#define ENCODER_B 4  // Encoder B pin
#define PWM_PIN 9    // Motor speed control
#define DIR_PIN 8    // Motor direction
#define OPEN_BUTTON 5 // Open button
#define CLOSE_BUTTON 6 // Close button
#define PIR_SENSOR 7  // PIR motion sensor
```
This section defines pin numbers and initializes key variables like encoder count and delay times.

### **2. Motor Speed Calculation**
```cpp
int calculateSpeed(int position) {
  int minSpeed = 80;
  int maxSpeed = 180;
  int slowZone = maxPosition * 0.2;
  int mediumZoneEnd = maxPosition * 0.8;
  int targetSpeed = (position < slowZone || position > mediumZoneEnd) ? minSpeed : maxSpeed;
  int speedStep = 5;
  if (currentSpeed < targetSpeed) currentSpeed = min(currentSpeed + speedStep, targetSpeed);
  else if (currentSpeed > targetSpeed) currentSpeed = max(currentSpeed - speedStep, targetSpeed);
  return currentSpeed;
}
```
This function smoothly adjusts motor speed based on door position.

### **3. Motor Control Functions**
```cpp
void moveMotor(int direction) {
  digitalWrite(DIR_PIN, direction);
  analogWrite(PWM_PIN, currentSpeed);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  currentSpeed = 0;
}
```
These functions control motor movement and ensure smooth acceleration/deceleration.

### **4. Main Loop Logic**
```cpp
void loop() {
  bool motionDetected = digitalRead(PIR_SENSOR) == HIGH;

  if (digitalRead(OPEN_BUTTON) == HIGH || motionDetected) {
    if (isClosing) { isClosing = false; isOpening = true; Serial.println("Retracting - Opening again..."); }
    else if (!isOpening) { isOpening = true; Serial.println("Opening..."); }
    else { closeDelay += 5000; Serial.print("Extending delay: "); Serial.println(closeDelay); }
  }
```
- If the **PIR sensor detects motion** or the **Open button is pressed**, the door starts opening.
- If **motion is detected again**, the auto-close delay is **increased by 5 seconds**.

```cpp
  if (digitalRead(CLOSE_BUTTON) == HIGH) {
    if (isOpening) { isOpening = false; isClosing = true; Serial.println("Manual Close Triggered..."); }
    else if (!isClosing) { isClosing = true; Serial.println("Closing..."); }
  }
```
- The **Close button manually overrides** the automatic operation.

### **5. Auto-Close Feature**
```cpp
  if (!isOpening && !isClosing && millis() - closeStartTime > closeDelay) {
    isClosing = true;
    Serial.println("Closing...");
  }
```
- After the set delay, the door automatically **starts closing**.

```cpp
  if (isClosing) {
    int speed = calculateSpeed(abs(encoderCount));
    moveMotor(LOW);
    if (encoderCount <= 0) {
      stopMotor();
      isClosing = false;
      closeDelay = baseCloseDelay;
      Serial.println("Door Fully Closed");
    }
  }
}
```
- If the door is closing but **motion is detected**, the door **retracts (opens again)**.

## **Safety Features**
✔ **Smooth Acceleration/Deceleration** - Prevents sudden jerks.  
✔ **Manual Override** - Open/Close buttons allow emergency control.  
✔ **PIR Sensor Integration** - Hands-free operation prevents unnecessary contact.

## **Possible Enhancements**
📌 **LED Indicators** - Show when motion is detected.  
📌 **Buzzer Alert** - Sounds when the door is closing for safety.  
📌 **Adjustable PIR Sensitivity** - Configurable for different environments.

## **Conclusion**
This project provides an efficient and smart automatic door control system using an Arduino, a PIR sensor, and motorized operation. The combination of manual buttons and automatic motion detection ensures **convenience, safety, and flexibility** for users. 🚀

