Here’s a complete documentation for your **Automatic Door Control** project, written in a clean, professional style with inline explanations, a use case story, and sample I/O behavior examples.

---

# 📘 Automatic Door Control – Full Documentation

## 🔧 Project Summary

This project controls a **motorized automatic glass door** using an **Arduino Uno**, based on **button input**, **limit detection via encoder count**, and **encoder feedback** for position control. It ensures smooth and safe door movement using **speed modulation**, **directional control**, and **safety mechanisms** like emergency stop and homing.

---

## ⚙️ Hardware Setup

### ➕ Components Used
| Component              | Pin Connections | Purpose                             |
|------------------------|-----------------|--------------------------------------|
| DC Motor               | PWM_PIN (9)     | Speed control                        |
| Direction Control      | DIR_PIN (8)     | Clockwise/Anticlockwise              |
| Rotary Encoder         | ENCODER_A (3)   | Counts motor steps (interrupt-based) |
|                        | ENCODER_B (4)   | Direction feedback                   |
| Open Button            | OPEN_BUTTON (5) | User input to open door              |
| Close Button           | CLOSE_BUTTON (6)| User input to close door             |
| Home Sensor (Limit)    | HOME (11)       | Detects full open position           |

---

## 🧠 Use Case Story

> **Imagine a modern office with an automatic glass door**. At the start of the day, power is turned on. The system doesn’t know the door’s current position, so it first runs a **Homing Procedure** to fully open the door. After that, it listens for button inputs:
>
> - When **Open** is pressed, the door opens unless it’s already fully open.
> - When **Close** is pressed, the door closes unless it’s already fully closed.
> - If **both buttons are pressed**, it's considered an emergency — the motor immediately stops.
> - During motion, the door **slows down near the endpoints** to prevent hitting hard stops or damaging the glass.
> - At any point, pressing a button while the opposite motion is in progress switches the operation cleanly (once current operation finishes or is stopped).
>
> Safety features include speed ramping, emergency stops, and encoder feedback-based hard stops. Auto-close is currently disabled during testing.

---

## 🧩 Core Concepts Explained

### 🚪 Encoder Count
- `encoderCount` tracks the door’s position.
- **Open Position**: `encoderCount = 0`
- **Closed Position**: `encoderCount ≈ 1050`

### 🏁 Homing
- At startup, the system performs homing by opening the door until the home sensor is triggered (LOW).
- Encoder count is reset to 0 once homing completes.

### 🧮 Motion Logic
- Motion starts when user presses the button.
- Speed is calculated based on the position:
  - Fast in middle zones.
  - Slower in near-end positions.
  - Slight torque boost at end of closing due to friction.

### 🛑 Emergency Stop
- Both buttons pressed at the same time? Motor stops immediately and sets `emergencyStopped = true`.

---

## 🔍 Function Breakdown

### `setup()`
- Initializes serial monitor, pins, and interrupts.
- Waits for homing on startup.

### `loop()`
Handles:
- Homing logic
- Open/Close button handling
- Emergency stop
- Speed-controlled motion using encoder
- Safe deceleration near endpoints
- Auto-close (currently disabled)

---

## 🧮 Speed Ramping Logic

```cpp
int calculateSpeed(int position, bool isOpeningDir)
```
- Returns PWM value dynamically adjusted:
  - Fast (180–200) in central zones.
  - Slow (80) near ends.
  - Slight boost (200) at the final zone of closing to overcome torque.
  - Gradual transitions using `speedStep = 5` to smooth motion.

---

## 📌 Sample Scenarios

| Scenario                     | System Behavior                                             |
|-----------------------------|-------------------------------------------------------------|
| Open button pressed          | Starts opening. Slows down near open end. Stops at 0.       |
| Close button pressed         | Starts closing. Boosts torque in final 5%. Stops at 1050.   |
| Open + Close pressed         | Emergency stop. Motor halts instantly.                      |
| Button pressed repeatedly    | Ignores if already in requested position.                   |
| Homing at boot               | Opens door slowly until home sensor is triggered.           |

---

## 🧪 Testing Considerations

- **Speed drop near ends** avoids glass door breakage.
- **Motor torque is increased only slightly** at the end of closing.
- **Auto-close** is disabled during testing for full manual control.
- **Real-time safety checks** like emergency stop are active.

---

## 🧼 Clean Serial Output Example

```
🔌 System Ready. Waiting for Homing...
✅ Homing Complete. Door is Fully Open.
🔽 Closing Door...
🔒 Door Fully Closed.
🔼 Opening Door...
✅ Door Fully Opened.
🛑 Emergency Stop: Both Buttons Pressed!
```

---

## 🧯 Safety & Edge Cases Handled

✔️ Homing on startup  
✔️ Emergency stop  
✔️ Slowing near ends  
✔️ Torque boost near end of closing  
✔️ Prevent movement beyond bounds  
✔️ Handles invalid button presses  
✔️ Smooth start/stop with ramping  
✔️ Auto-close isolated cleanly for testing  

---

## ✅ Final Notes

- The code is **hardware-safe**: never pushes glass with excess force.
- **Modular speed logic** makes it easy to tune later.
- Can be extended for **automatic sensors**, **wireless control**, or **timed logic**.
- Robust for real-world deployment.

---

Would you like a PDF version of this documentation? Or shall I help you integrate this into a report/research paper format?
