**Closed-Loop 3D Printing Filament Extrusion Controller**

This repository contains the software and design files for a closed-loop measurement and control system built for an EX6 Filabot filament extruder. The system continuously monitors the diameter of extruded 3D-printing filament and uses a PID controller to dynamically regulate the pulling motor speed, ensuring uniform material thickness.

Instead of relying on expensive optical sensors, this project utilizes a custom-designed mechanical gauge that couples filament thickness variations to a magnet. A linear Hall-effect sensor reads the magnetic displacement, which is then digitized, calibrated to millimeters via a polynomial transfer function, and processed by a Raspberry Pi 5 to apply real-time motor corrections.

**Project Components:**

* **Hardware:** Raspberry Pi 5, SS49E linear Hall-effect sensor, ADS1115 16-bit ADC (I2C), and an L298N motor driver

* **Software & Control:** Python-based PID controller featuring dead-time compensation, anti-windup clamping, and a Tkinter graphical user interface (GUI) for real-time monitoring and parameter tuning

* **Mechanical & PCB Design:** A custom 3D-printed bearing gauge modeled in Autodesk Fusion and a dedicated sensor PCB designed in Proteus