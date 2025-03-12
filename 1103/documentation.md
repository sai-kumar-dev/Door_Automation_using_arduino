Documentation
---

# 🧾 **Automatic Door Control System - Documentation**

## 📌 Overview

This is a smart **automatic glass door controller** implemented using **Arduino**. It controls a door with:
- A **DC motor** (for opening/closing),
- **Quadrature encoder** (for position tracking),
- **Limit switch** (homing reference),
- Two push-buttons (**open** and **close**),
- And PWM-based **speed control** for torque-sensitive smooth movement.

---

## 🛠️ **Hardware Interface**

| Pin       | Function                 |
|-----------|--------------------------|
| 3         | Encoder A (Interrupt)    |
| 4         | Encoder B                |
| 8         | Motor Direction          |
| 9         | PWM to control Motor     |
| 5         | Open Button (Input Pullup)|
| 6         | Close Button (Input Pullup)|
| 11        | Homing Sensor (Input Pullup)|

---

## 🚦 **System Behavior & Use Case Story**

### 📖 Use Case Story

> Imagine a glass office door. Initially, it's in an unknown position. On powering up, the system performs a **homing routine**: it gently moves the door until it reaches the "fully open" position using the home switch. After homing:
>
> - If someone presses **"Close"**, the door smoothly **accelerates to max speed**, then **decelerates** as it nears full close.
> - If someone presses **"Open"**, the reverse happens.
> - **Both buttons pressed together** triggers an **emergency stop** (for safety).
>
> ⚠️ Torque required near closed state is higher, so we use **gradual deceleration** to prevent the motor from stalling or jerking. This protects the **glass** and prevents gear/motor damage.

---

## 💡 Key Features and Choices

### 1. ✅ **Homing Routine**
- Required because encoder has **no absolute position** on startup.
- Home switch is mounted to detect fully open position.
- Encoder is reset to 0 upon reaching home.

### 2. 🧠 **Stateful Handling**
- Boolean flags `isOpening`, `isClosing` ensure **mutual exclusivity**.
- Prevents multiple simultaneous actions or redundant operations.

### 3. 🔁 **Speed Ramping Logic**

**Why Smooth Speed Transition?**
- DC motors require **torque** to start, especially against friction (like door seals).
- If speed instantly jumps from 0 to high or high to 0, it can:
  - Stall the motor,
  - Damage the door or motor,
  - Be noisy or unsafe.

**Our Solution:**
- Speed changes **step-by-step** (`speedStep = 3`) toward a target value.
- Deceleration is **position-aware**, so door slows down **only near the end**.
- Near full open or full close, speed reduces from `100 ➝ 30` to gently land.

**Analogy:**
> Like how a train approaches a station—it doesn’t slam the brakes, it slows down gradually based on how far it is.

### 4. 🛑 **Emergency Stop Logic**
- Both buttons pressed? That’s not normal.
- We stop the motor immediately and log the event.

### 5. ⏳ **Auto-Close (Temporarily Disabled)**
- After remaining open for a fixed time, door can auto-close.
- This is currently **commented out** for testing convenience.

---

## 📐 Constants & Configurations

| Name               | Value  | Purpose                           |
|--------------------|--------|-----------------------------------|
| `maxPosition`      | 1050   | Encoder count when fully closed   |
| `baseCloseDelay`   | 5000   | Auto-close wait time (ms)         |
| `maxMotionDuration`| 7000   | Safety timeout per motion (ms)    |
| `minSpeed`         | 30     | Speed near door end               |
| `maxSpeed`         | 100    | Max door movement speed           |
| `rampStartZone`    | 150    | When deceleration starts          |
| `rampThreshold`    | 50     | Where to keep min speed constant  |

---

## 🧪 Example Scenarios

### ✅ Homing at Startup
```bash
🔌 System Ready. Waiting for Homing...
✅ Homing Complete. Door is Fully Open.
```

### 🔽 Closing Door
```bash
🔽 Closing Door...
🔒 Door Fully Closed.
```

### 🔼 Opening Door
```bash
🔼 Opening Door...
✅ Door Fully Opened.
```

### 🛑 Emergency Stop
```bash
🛑 Emergency Stop: Both Buttons Pressed!
```

---

## 🚧 Edge Case Handling

| Scenario                            | Handled? | Notes                          |
|-------------------------------------|----------|--------------------------------|
| Pressing open when already open     | ✅       | Message shown                  |
| Pressing close when already closed  | ✅       | Message shown                  |
| Both buttons pressed                | ✅       | Emergency stop                 |
| Encoder stuck / motion stalled      | ✅       | Timeout abort after 7s         |
| Door overshoots encoder count       | ✅       | Liberal use of `<=` and `>=`   |
| Auto-close needed?                  | ✅(temp off) | Can be re-enabled easily  |

---

## 🔍 Why Not Use Fixed Speed?

> In real-world mechanical systems like sliding glass doors, **torque varies with position**.
- Near end points, friction is high.
- High speed near end = **glass break risk**.
- Low speed start = **can’t overcome friction**.

Hence, we chose **gradual acceleration/deceleration** using encoder position feedback.

---

## 🧠 Suggested Improvements (Future Scope)
- Add buzzer or LED for visual alerts.
- Add load sensing to detect motor stall.
- Support for **infrared** or **touchless** activation.
- EEPROM store for encoder count if homing fails.

---

## 🧾 Summary

This system provides:
✅ Reliable movement  
✅ Torque-safe ramping  
✅ Position-aware motion  
✅ Safety checks  
✅ Clean serial feedback for debugging  

Built with real-world physical constraints in mind, this design blends **hardware logic** with **software adaptability**, ensuring safety, reliability, and maintainability.

---
