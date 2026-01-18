# Driver Drowsiness Detection System  
**Embedded C | ESP32 | IMU | ECG**

## 📌 Overview
The **Driver Drowsiness Detection System** is an embedded safety solution designed to detect early signs of driver fatigue using physiological and motion-based signals.  
The system combines **head movement analysis** from an IMU sensor and **heart rate monitoring** from an ECG sensor to identify potentially hazardous driver states and trigger real-time alerts.

This project demonstrates practical implementation of **sensor fusion**, **real-time signal processing**, and **embedded firmware development** for automotive safety applications.

---

## ⚙️ System Architecture
- **Microcontroller:** ESP32  
- **Motion Sensing:** MPU6050 (3-axis accelerometer + gyroscope)  
- **Physiological Monitoring:** ECG sensor  
- **Alert Mechanism:** Buzzer / LED  
- **Programming Language:** Embedded C  

---

## 🧠 Working Principle
1. **Head Tilt Detection**  
   - MPU6050 continuously monitors pitch and roll angles.
   - Abnormal or prolonged head tilt indicates possible drowsiness.

2. **Heart Rate Monitoring**  
   - ECG sensor captures heart rate signals.
   - Sudden drops or irregular patterns are analyzed as fatigue indicators.

3. **Sensor Fusion Logic**  
   - IMU and ECG data are processed together to improve detection accuracy.
   - False positives are reduced by correlating motion and physiological data.

4. **Alert Generation**  
   - When fatigue thresholds are exceeded, an immediate alert is triggered to warn the driver.

---

## 🚀 Features
- Real-time drowsiness detection  
- Sensor fusion for improved reliability  
- Low-latency alert system  
- Embedded, standalone operation  
- Suitable for automotive safety research  

---

## 📂 Project Structure
```text
driver-drowsiness-detection-esp32/
│── src/
│   ├── main.c          
│   
│── .gitignore
│── README.md
