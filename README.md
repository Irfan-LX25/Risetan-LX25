# Ri-Setan-LX25
LX25 PROGRAMMING RESEARCH AND DEVELOPMENT

Dokumentasi Riset Program LX24: https://github.com/HabibMuhammad05/KRAI-ESPNOW-DS2-CONTROLLER-PROJECT

    -DS2 controller using ESP32 with the ESP-NOW communication protocol
    -Includes all features for transmitter and receiver
    -R1 && R2 Abu Robocon 2025 Programming Research
    -PG45 Motor PID Control Using Internal Encoder – Research
    
    



<div align="center">

# 🤖 RI-SETAN LX25  

<img src="docs/KRAI-01.png" width="180"/>


![GitHub repo size](https://img.shields.io/github/repo-size/Irfan-LX25/Ri-Setan-LX25)
![GitHub last commit](https://img.shields.io/github/last-commit/Irfan-LX25/Ri-Setan-LX25)
![Robotics](https://img.shields.io/badge/Robotics-Research-blue)
![Embedded](https://img.shields.io/badge/Embedded-System-green)

</div>

---

## 🏆 About The Team

**RI-SETAN LX25** adalah divisi riset dan pengembangan sistem kendali robot yang berfokus pada:

- ⚙️ Motion Control & PID
- 🕹️ Joystick & Human Interface
- 🔁 Dual-Mode Control System
- 🤖 Mobile Robot (Swerve / Differential)
- 🧪 Experimental Embedded Programming

Repositori ini digunakan sebagai **bank riset internal & dokumentasi teknis**.

---

## 🎯 Project Objectives

- Mengembangkan sistem kontrol robot yang stabil dan presisi
- Membangun modul joystick yang responsif dan modular
- Melakukan tuning PID berbasis eksperimen
- Menyediakan dokumentasi teknis untuk pengembangan berkelanjutan

---

## 🧩 System Overview

```mermaid
flowchart LR
    Operator --> Joystick
    Joystick --> Controller[LX25 Controller]
    Controller --> MotorDriver
    MotorDriver --> Motor
    Encoder --> Controller
