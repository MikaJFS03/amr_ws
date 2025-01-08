# Edabot Bringup Package

## Overview
Package ini berisi dua buah launch file untuk menjalankan keseluruhan sistem robot. Kedua file yang dimaksud terdiri dari: (1) Simulation Launch dan (2) Robot Hardware System. Masing-masing terdiri dari bagian berikut:
* Interface Launch: berisi sistem interface robot yang terdiri dari Gazebo untuk simulasi dan hardware interface untuk robot secara langsung.
* Controller Launch: berisi sistem kendali dengan jenis differential drive berbasis ROS2 Control, serta twist multiplexer untuk kombinasi dan pengaturan prioritas kecepatan sistem.
* Teleoperation Launch: berisi sistem teleoperasi yang terdiri dari joystick/gamepad (untuk teleoperasi langsung), maupun terhubung ke dashboard berdasarkan topik yang sama
* SLAM Launch: menjalankan sistem Simultaneous Localization and Mapping (SLAM) yang terdiri dari pemetaan dan lokalisasi pada robot dengan termasuk ZED2i Camera
* Navigation Launch: menjalankan sistem navigasi robot berbasis pada ROS2 Navigation2 Stack (Nav2)
* Vision Launch: (coming soon)

## Penggunaan
Cara menjalankan robot dengan sistem bringup (secara keseluruhan) adalah sebagai berikut:
```bash
ros2 launch edabot_bringup robot.launch.py # untuk robot

# atau

ros2 launch edabot_bringup simulation.launch.py # untuk simulasi
```