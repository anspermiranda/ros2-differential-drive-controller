# ROS2 Differential Drive Robot Controller

## 📌 Overview

This project implements a **ROS2-based control system for a differential drive mobile robot**.

The system simulates a robot that performs a complete motion task:

1. Rotate to a specified angle
2. Move forward a fixed distance
3. Stop automatically

The implementation demonstrates how robot motion can be controlled using **ROS2 nodes, topics, and services**, along with kinematic modelling.

---

## 🎯 Objectives

* Design a multi-node ROS2 system
* Implement robot motion control using wheel velocities
* Apply differential drive kinematics
* Use ROS2 communication (topics and services)

---

## ⚙️ System Architecture

The system is composed of three ROS2 packages:

### 🔹 Robot Simulation (`digital_enforcer_pkg`)

* Simulates robot motion
* Updates position using kinematics
* Publishes robot pose (x, y, θ)

---

### 🔹 Controller (`digital_enforcer_controller_pkg`)

* Controls robot behaviour
* Implements:

  * Rotation phase
  * Forward motion phase
* Publishes wheel velocities

---

### 🔹 Launch Package (`digital_enforcer_launch_pkg`)

* Launches the full system
* Connects all nodes together

---

## 🧠 Core Concepts

### Differential Drive Kinematics

The robot motion is governed by:

* Linear velocity:
  v = (r/2)(ωr + ωl)

* Angular velocity:
  ω = (r/l)(ωr - ωl)

Where:

* r = wheel radius
* l = distance between wheels

These equations are used to compute robot movement in real time.

---

## 🔄 Control Strategy

The controller operates in sequential phases:

1. **Rotate Phase**

   * Align robot to target orientation

2. **Forward Phase**

   * Move straight for a defined distance

3. **Stop Phase**

   * Stop robot and terminate motion

---

## 📡 ROS2 Communication

### Topics

* `/digital_enforcer/task_space_pose`
* `/digital_enforcer/wheel_angular_velocities`

### Services

* `/digital_enforcer/turn_robot_on`
* `/digital_enforcer/turn_robot_off`

---

## 📂 Project Structure

```
ros2-differential-drive-controller/
│
├── src/
│   ├── digital_enforcer_pkg/
│   ├── digital_enforcer_controller_pkg/
│   └── digital_enforcer_launch_pkg/
│
├── docs/
│   └── coursework.pdf
│
└── README.md
```

---

## ▶️ How to Run

### 1. Install ROS2 (Jazzy recommended)

```bash
source /opt/ros/jazzy/setup.bash
```

---

### 2. Build the workspace

```bash
colcon build
```

---

### 3. Source the workspace

```bash
source install/setup.bash
```

---

### 4. Launch the system

```bash
ros2 launch digital_enforcer_launch_pkg digital_enforcer_launch.py
```

---

## 📊 Expected Behaviour

* Robot starts in OFF state
* Controller turns robot ON
* Robot rotates to target angle
* Robot moves forward
* Robot stops automatically

---

## 📚 Learning Outcomes

This project demonstrates:

* ROS2 architecture (nodes, topics, services)
* Robot kinematics and motion modelling
* Control system implementation
* Multi-package project structure
* Real-time robotics software design

---

## 🚀 Future Improvements

* Add PID control for smoother motion
* Integrate with Gazebo simulation
* Add obstacle avoidance
* Combine with A* path planning

---

## 🏫 Academic Context

Developed for:
**Software for Robotics – Coursework**

---

## 👤 Author

Ansper Miranda
