# Edabot Controller Package

## Overview
Package ini berisi kontrol motion robot, teleoperasi, dan pengaturan prioritas kecepatan. Robot dikendalikan dengan sistem differential drive melalui plugin diff_drive_controller. Sistem ini mengubah kecepatan linear dan angular robot menjadi kecepatan untuk masing-masing (kedua) motor/roda. Konfigurasi untuk sistem kontrol ini dapat diatur pada file konfigurasi amar_controller.yaml pada bagian amar_controller. Sistem teleoperasi utama dikendalikan dengan basis kontroler berupa joystick yang dapat diatur pada file konfigurasi joy_config.yaml dan joy_teleop.yaml dengan topik /cmd_vel_joy. Beberapa input kecepatan yakni dari teleoperasi maupun navigasi dikombinasikan menjadi satu berdasarkan prioritas tertentu melalui node twist_mux dari package twist_mux yang dapat diatur pada file konfigurasi twist_mux.yaml.

## Penggunaan
Cara menjalankan sistem  adalah sebagai berikut:
```bash
ros2 launch edabot_communication communication.launch.py
```

## Others
Berikut cara melakukan teleoperasi melalui keyboard user.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/input_key/cmd_vel_stamped
```