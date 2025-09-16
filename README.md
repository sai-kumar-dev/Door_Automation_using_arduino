# NFC-Project
# 🚪 Automated Door System Using Arduino

## 📘 Overview
This project demonstrates an **automated door system** controlled using an **Arduino UNO**, sensors, and a DC motor.  
The system is designed to **automatically open and close a door** based on proximity detection, with position control using an encoder.  
This project was developed as part of **NFC Mind Hackathon / Workshop**.

---

## 🏗️ Project Components

- **Arduino UNO** – Microcontroller to control the system
- **100W DC Motor** – Drives the door
- **PWM Motor Controller** – Controls motor speed and direction
- **Encoder (4-pin)** – Tracks door position
- **Inductive Proximity Sensor** – Homing/reference position
- **PIR Sensor** – Detects presence to open/close door
- **Electromagnetic Relay** – Controls power to motor
- **SMPS (Power Supply)** – Powers motor controller
- **Jumper Wires** – For connections

---

## ✨ Features

- Automatic door opening and closing using **PIR sensor detection**
- **Homing functionality** via inductive proximity sensor
- **Position tracking** using motor encoder pulses
- **PWM motor control** for speed and direction
- Manual override possible through Arduino programming
- Safe operation with relay-controlled power

---

## 🛠️ How It Works

1. The **PIR sensor** detects a person approaching.
2. The Arduino calculates the required motor motion and sends **PWM signals** to the motor controller.
3. The **encoder** provides feedback to track door position.
4. The **proximity sensor** sets the home position for accurate movement.
5. The **electromagnetic relay** manages the main power flow safely.
6. Door closes automatically after a preset delay.

---
Project structure 

## 📂 Project Structure
Date Wise Updates :
0403
0603
0703
0803
1103
1203
1303
1503
1703
1803
1903
2003
2103  Project Report
my_code
 1203
6 months ago
Sir_code
ABHI & SAI.docx 1703 
Thala version.docx
 1703 Documentation 
all copy for hr pr.docx
1703 
nfc_project_doc.docx

---

## ⚙️ Setup Instructions

1. Connect all sensors, motor, and relay to the Arduino UNO as per the schematics.
2. Upload the Arduino sketch (`.ino`) file from `Arduino_Code/`.
3. Power the motor controller using the SMPS.
4. Ensure encoder and proximity sensor connections are correctly wired.
5. Test the system by walking in front of the PIR sensor — the door should open and then close automatically.

---

## 👨‍💻 Team Members

- **Abhishek Varma** –  Developer / Collaborator  
- **Garlapati sai Kumar** – Developer / Team Lead  
- **T Kusuma Reddy** – Developer / Collaborator  

---

## 📌 Future Improvements

- Add **RFID/NFC-based access control**  
- Integrate **IoT monitoring** to track door status remotely  
- Add **manual override via mobile app**  

---

## 📌 References

- Arduino official documentation: [https://www.arduino.cc](https://www.arduino.cc)  
- PWM motor control tutorials  
- Sensor datasheets for PIR and proximity sensors

