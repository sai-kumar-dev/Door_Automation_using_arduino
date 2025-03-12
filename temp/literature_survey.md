Alright! Let’s craft a **detailed literature survey** tailored for your project:

---

## 📚 **Literature Survey**

---

### 🔹 **1. Overview**

The automatic door control system is a fundamental part of modern automation in smart buildings, hospitals, public facilities, and industrial zones. Over the years, various approaches have been proposed for implementing intelligent door systems. These include sensor-based, wireless, IoT-enabled, and motor control techniques. This literature survey explores key research contributions and existing methodologies related to automatic doors with a focus on motor control, encoder feedback, homing mechanisms, and safety features like emergency stops and smooth motion management.

---

### 🔹 **2. Existing Work in Automatic Door Systems**

#### 📄 **2.1 Sensor-Based Automatic Door Systems**

Most early systems focused on **motion detection or IR sensors** to trigger door operations.  
**Example:**  
> *“Design and Implementation of an Automatic Door System using PIR Sensor” - IJERT, 2015*  
This system opened the door when motion was detected using a Passive Infrared (PIR) sensor. It lacked real-time position feedback and closed the door based on a static delay.

**Limitations:**
- No control over motor position
- Inaccurate in high-traffic areas
- No emergency stop functionality

---

#### 📄 **2.2 Timer-Based and Relay-Driven Doors**

> *“Microcontroller Based Automatic Door Opening System” – IJSER, 2017*  
This paper implemented door opening and closing using relays with time delays for closing. It used pushbuttons for manual triggers but no encoders or feedback systems.

**Limitations:**
- Rigid delay timings
- Lacked real-world feedback (like door position)
- Poor smoothness during operation

---

### 🔹 **3. Use of Encoders in Automation**

Encoder integration improves positional accuracy, enabling **precise movement control and speed regulation**.

> *“Speed and Position Control of DC Motor Using Encoder and PID Controller” – IJERT, 2021*  
This study used encoder feedback and PID control to manage DC motor speed in a robotic arm. It proved that encoder feedback significantly improves motor handling.

**Relevance to Our Work:**
- Encoder helps detect the door’s position
- Supports emergency stops based on real-time position
- Enables smooth landing logic by tracking speed and position

---

### 🔹 **4. Homing Mechanism and Position Calibration**

Homing is a **critical process** in electromechanical systems to establish a known reference point.

> *“Design and Control of a Homing Mechanism for CNC Applications” – IEEE Mechatronics, 2019*  
The paper discussed how systems return to a 'home' position using limit switches. The logic ensures all movement is relative to a known start.

**Relevance to Our Work:**
- Our home sensor defines door's zero position
- Encoder is reset here for accurate travel measurement
- Important for consistent behavior and safe operations

---

### 🔹 **5. Smooth Acceleration and Deceleration**

To avoid jerks or mechanical stress, smooth motion is essential.  
> *“Soft Start/Stop DC Motor Control Using PWM and Encoder Feedback” – IJRASET, 2020*  
This work shows how to adjust PWM signals to gradually change motor speed using encoder feedback to detect motion or stalling.

**Application to Our Project:**
- Door starts with low speed
- Increases if not moving (for overcoming inertia)
- Decreases speed near endpoints for smooth stop

---

### 🔹 **6. Emergency Stop and Safety Features**

> *“Design of a Safe Elevator Door System with Obstruction Detection and Emergency Stop” – IEEE Sensors, 2022*  
Elevators require doors that can stop immediately when safety is at risk. The logic includes both physical sensors and logic-based emergency conditions.

**Relevance:**
- Emergency stop if opposite button is pressed during motion
- Interrupt current operation without damaging hardware
- Adds critical safety and manual override capability

---

### 🔹 **7. Manual Override and Auto-Close Features**

> *“Hybrid Manual-Auto Smart Door using Arduino” – IRJET, 2019*  
This system offered both automatic motion detection and manual push-button controls. It also implemented auto-close delay but lacked encoder-based precision.

**Takeaway:**
- Combining manual (buttons) + automatic (delay-based) adds flexibility
- Our design extends it with position awareness and smooth speed transitions

---

### 🔹 **8. Summary and Research Gap**

| Feature                         | Traditional Systems | Our System |
|----------------------------------|---------------------|------------|
| Motion Sensor/Trigger            | ✅                  | ❌         |
| Button Manual Controls           | ⚠️ (Basic)           | ✅         |
| Encoder-Based Feedback           | ❌                  | ✅         |
| Homing/Calibration Mechanism     | ❌                  | ✅         |
| Smooth Speed Ramping             | ❌                  | ✅         |
| Emergency Stop Logic             | ❌                  | ✅         |
| Real-time Serial Debugging       | ⚠️                  | ✅         |

---

### 🔹 **9. Conclusion of Literature Survey**

While many existing systems use basic logic for opening/closing doors, most **lack precision, feedback, and smart control mechanisms**. Our approach fills these gaps by:

- Incorporating **real-time encoder feedback**
- Ensuring **homing** for consistent initialization
- Enabling **manual control** with logical overrides
- Applying **dynamic speed adjustments** for safety
- Adding **debugging and transparency** through serial communication

This project not only improves the practical usability of an automatic door system but also serves as a **research-grade embedded systems project** ideal for educational, industrial, and smart home applications.

---

Would you like this to be:

- Formatted into a **LaTeX IEEE-style PDF**?
- Converted into **slides** for your viva or seminar?
- Added as a **chapter** in your full project report?

Let’s go next-level with this! 😎
