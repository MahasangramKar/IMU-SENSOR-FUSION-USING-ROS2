🚀 Real-Time IMU Sensor Fusion using ROS 2
📌 Overview

This project implements real-time sensor fusion of IMU data using ROS 2 to estimate accurate orientation (roll, pitch, yaw).

It combines data from:

Accelerometer
Gyroscope (optional: Magnetometer)
to provide stable and drift-free orientation for robotics and autonomous systems.

🎯 Objectives
Perform real-time IMU data acquisition
Implement sensor fusion algorithms
Publish filtered orientation using ROS 2 topics
Visualize orientation in RViz
🧠 Sensor Fusion Concept

IMU sensors individually have limitations:

Accelerometer → Noisy but stable long-term
Gyroscope → Smooth but drifts over time

👉 Sensor fusion combines both for accurate estimation.

🏗️ System Architecture
IMU Sensor → ROS 2 Sensor Node → Fusion Node → /imu/data → RViz Visualization

📦 Nodes
imu_node
Reads raw IMU data
Publishes /imu/raw
fusion_node
Applies sensor fusion algorithm
Publishes /imu/data


⚙️ Tech Stack
ROS 2 (Humble / Foxy)
Python / C++
IMU Sensor (MPU6050 / MPU9250)
RViz for visualization
