🚀 IMU Sensor Fusion using ROS 2






📌 Overview

This project implements real-time IMU sensor fusion using ROS 2 to estimate accurate orientation (roll, pitch, yaw).

It combines data from:

Accelerometer
Gyroscope
(Optional) Magnetometer

to achieve stable and drift-free orientation for robotics applications.

🎯 Objectives
📡 Acquire real-time IMU data
🧠 Implement sensor fusion algorithms
📤 Publish filtered orientation using ROS 2 topics
📊 Visualize results in RViz
🧠 Sensor Fusion Concept

IMU sensors have limitations:

Accelerometer → Noisy, stable long-term
Gyroscope → Smooth, but drifts

👉 Fusion combines both for accurate estimation.

🏗️ System Architecture

IMU Sensor
   ↓
ROS 2 Sensor Node
   ↓
Fusion Node
   ↓
/imu/data
   ↓
RViz Visualization

📦 ROS 2 Nodes
🔹 imu_node
Reads raw IMU data
Publishes: /imu/raw
🔹 fusion_node
Applies fusion algorithm
Publishes: /imu/data

⚙️ Tech Stack
ROS 2 (Humble / Foxy)
Python / C++
IMU: MPU6050 / MPU9250
RViz
