# Edabot Communication Package

## Overview
Package ini berisi sistem komunikasi dengan Modul Komunikasi. Beberapa source code yang diterapkan pada modul SLAM berbasis modul Komunikasi adalah sebagai berikut:
* friend_pose: berfungsi untuk subscribe posisi robot teman dan dimasukkan ke dalam peta melalui bentuk pointcloud sehingga dapat dihindari dalam proses navigasi
* navigate_to_goal_action_client: berfungsi untuk meneruskan nilai koordinat tujuan robot yang diterima melalui Modul Komunikasi dan memanggil action untuk sistem path dan waypoint following

## Penggunaan
Cara menjalankan sistem  adalah sebagai berikut:
```bash
ros2 launch edabot_communication communication.launch.py
```