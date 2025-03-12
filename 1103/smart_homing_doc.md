---

# 🛠️ Automatic Door Control System – Documentation

## 📄 Overview

This system controls an automatic glass sliding door using:
- **Encoder-based position tracking**
- **Button-controlled opening/closing**
- **Smart homing**
- **Smooth motion control with speed ramping**
- **Failsafes like emergency stop and timeout**

Built on an Arduino Uno, it prioritizes **real-world reliability**, **safety**, and **durability of the glass mechanism** — with intelligent speed handling to avoid harsh movements or jams.

---

## ⚙️ Hardware Used

| Component           | Purpose                                      |
|---------------------|----------------------------------------------|
| Arduino Uno         | Main controller                              |
| DC Motor + Driver   | Moves the door                               |
| Rotary Encoder      | Tracks door position                         |
| Limit Switch (HOME) | Detects fully open door for homing           |
| Open Button         | Triggers door opening                        |
| Close Button        | Triggers door closing                        |
| Glass Door          | The actual controlled object                 |

---

## 🔍 Key Concepts & Features

### 1. 🔁 **Encoder-based Position Tracking**
The encoder increments/decrements the position (`encoderCount`) when the motor moves.  
- **Fully Open:** `encoderCount = 0`
- **Fully Closed:** `encoderCount ≈ 1050`

> **Analogy:** Like a measuring tape that counts how far a drawer has opened or closed.

---

### 2. 🏠 **Smart Homing Logic**

**Why?**  
Glass doors can break if slammed into a hard stop. Standard homing doesn't consider torque. So we built a **"feel-as-you-go" homing** strategy.

**How it works:**  
- Starts at a **low speed**.
- If no movement is detected via encoder, the system **increases speed** gradually.
- Once movement starts, it **slows down** again to catch the sensor gently.
- When HOME sensor triggers (LOW), stop and reset position to 0.

> **Real-life Analogy:** Imagine gently pushing a stuck drawer — increase force slowly till it moves, then ease off before it hits the back.

**Why this approach?**
- Prevents overshooting the home sensor.
- Adapts to door load, friction, or gravity.
- Works without external feedback other than encoder.

---

### 3. 🎚️ **Smooth Speed Ramping**

**Problem Solved:**  
Motors can stall at low speeds (especially while closing), but accelerating or decelerating too fast risks breaking the door.

**Solution:**  
- Dynamic speed adjustment based on current position.
- During:
  - **Opening** → Speed decreases from 100 ➝ 30 gradually.
  - **Closing** → Same smooth ramp-down.
- Encoder position is used to decide how far from the end position the door is, and the speed is adjusted accordingly.

> **Analogy:** Think of an elevator — it starts fast, then slows down before stopping on your floor.

---

### 4. 🚨 **Emergency Stop**

**Condition:** If both buttons are pressed simultaneously while moving.

**Effect:**  
- Motor stops immediately.
- Movement is cancelled.
- Prints debug message for diagnosis.

> **Use case:** Human override or unexpected physical obstruction.

---

### 5. ⏱️ **Timeout Protection**

**Problem:** Motor or encoder might fail, risking damage or infinite loop.

**Solution:**  
- If opening/closing takes more than 7 seconds → force stop.

> **Analogy:** Like an oven timer — if it takes too long, something must be wrong.

---

### 6. 🧪 **Auto-Close (Temporarily Disabled)**

**Purpose:** Close the door automatically after a delay.

**Status:** Disabled during testing for manual control only.

---

## ✅ Case-Based Behavior Explanation

| Case | Scenario | Behavior | Debug Messages |
|------|----------|----------|----------------|
| 1 | Door is open and OPEN is pressed | No action | `ℹ️ Door is already open.` |
| 2 | Door is closed and CLOSE is pressed | No action | `ℹ️ Door is already closed.` |
| 3 | Door is closing and OPEN is pressed | Emergency Stop | `🛑 Emergency Stop: Both Buttons Pressed!` |
| 4 | Door is opening and CLOSE is pressed | Emergency Stop | Same as above |
| 5 | Encoder fails during movement | Timeout failsafe | `✅ Door Fully Opened.` or `🔒 Door Fully Closed.` after timeout |
| 6 | Homing at low speed fails | Increases speed till movement | Logs speed adjustment and completes homing |
| 7 | High-speed crash risk | Avoided by smooth ramping | Speed auto-adjusts safely |

---

## ⚡ Example Use Flow

### Example 1: Initial Power On
1. Arduino boots up.
2. Homing is performed using smart feedback.
3. Door is now known to be open.

### Example 2: Someone presses CLOSE
- Motor moves door slowly at first, gains speed, then ramps down before fully closing.

### Example 3: Something blocks door and both buttons are pressed
- Motor instantly stops.
- Debug message is shown.
- Door won't resume unless triggered again.

---

## 🔍 Why This Design?

| Challenge | Solution | Reasoning |
|----------|----------|-----------|
| Encoder missing steps during homing | Smart feedback-based speed adaptation | Reacts dynamically to door resistance |
| Glass break risk | Smooth speed control | Safety and durability |
| Motor not moving at low speed | Adaptive start-up logic | Ensures torque is enough to move |
| Real-time constraints | No external CPU required | Runs standalone on Arduino |
| Redundant output in logs | Clean, single-line updates | Easy debugging and diagnosis |

---

## 🧠 Final Thoughts

This system is built with **embedded safety**, **adaptive logic**, and **real-world practicality** in mind.  
It doesn't just assume — it **checks**, **adapts**, and **reacts**.

---
