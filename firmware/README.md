# ⚙️ Firmware – Educational Speed Robot

This directory contains the **embedded firmware** for the Educational Speed Robot project.  
It runs on an **ESP32 microcontroller**, handling **PID-based motor control**, **line tracking**, and **Bluetooth telemetry**.

---

## 📄 Overview

The firmware was written in **C++ (Arduino framework)** and refactored for clarity and presentation in this public portfolio repository.  
It demonstrates:
- Real-time control loops and encoder-based feedback
- Modular C/C++ firmware structure (`src/` + `include/`)
- BLE command interface for runtime tuning
- QTR sensor array line detection and correction logic

---

## 📁 Folder Structure

| Path | Description |
|------|--------------|
| `src/SpeediBot_refactored.ino` | Main firmware logic (control loop, BLE, PID) |
| `include/config.h` | All pin mappings, constants, and BLE UUIDs |
| `README.md` | Overview and usage notes for firmware |
| `.gitignore` | Excludes build artifacts (if built later) |

---

## 🔧 Highlights

- **Controller:** ESP32 DevKitC  
- **Control Scheme:** PI loop with encoder feedback  
- **Sensors:** QTR-8RC line sensor array  
- **Motor Driver:** L298N dual H-bridge  
- **BLE Commands:** `SPEED=0.8`, `START`, `STOP`  
- **Max Speed:** ~0.8 m/s (98% tracking accuracy on 10 m course)

---

## 🔤 BLE Command Interface

| Command | Description |
|----------|-------------|
| `START` | Begin robot motion |
| `STOP` | Halt all motion |
| `SPEED=<m/s>` | Set target linear speed |
| *(optional)* `KP=<value>` / `KI=<value>` | Adjust PID gains live (future use) |

---

## 🧠 Control Loop Summary

```text
[ QTR Sensors ] → [ Line Error ] → [ PID Controller ] → [ PWM Outputs ]
                                          ↑
                                  [ Encoder Feedback ]
