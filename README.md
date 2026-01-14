# 🧠 Multi-Robot Fleet Manager (ROS 2 | Gazebo | TurtleBot3)

A **simulation-based multi-robot fleet management system** built using **ROS 2**, demonstrating **scalable robot coordination, quorum-based readiness, and deterministic robot selection**.

This project focuses on **fleet-level control logic** rather than single-robot navigation.

---

## 🚀 Project Overview

This project implements a **centralized fleet controller** that:

- Manages **25 TurtleBot3 robots** in a Gazebo simulation
- Subscribes to odometry from all robots
- **Waits until all robots are ready** before making decisions
- Selects the **nearest robot to a goal**
- Uses a **PID-based controller** to drive the selected robot
- Ensures **only one robot moves at a time**
- Demonstrates **production-grade ROS 2 design patterns**

This architecture closely resembles **industrial AMR fleet managers** used in warehouses and factories.

---

## 🎯 Key Features

- ✅ Supports **25 robots simultaneously**
- ✅ Uses **ReentrantCallbackGroup** to avoid callback starvation
- ✅ Uses **MultiThreadedExecutor** for scalability
- ✅ Correct **QoS (BEST_EFFORT)** for Gazebo odometry
- ✅ **Simulation time synchronization** (`use_sim_time`)
- ✅ **Hard quorum gating** – no robot selection until all robots publish odom
- ✅ **Readiness latch** to prevent race conditions
- ✅ Deterministic **nearest-robot selection**
- ✅ Explicitly stops all non-active robots

---

## 🛠️ Tech Stack

- ROS 2 (Python)
- Gazebo
- TurtleBot3
- rclpy
- MultiThreadedExecutor
- ReentrantCallbackGroup

---


---

## ▶️ How to Run

###  Launch the multi-robot simulation
```bash
ros2 launch turtlebot3_multi_robot multi_robot.launch.py
```
###  Run python Ros2 node
```bash
python3 multi_robot_fleet_manager.py
```

## 🚧 Future Improvements

- 🔁 Support multiple robots moving simultaneously  
- 🗺️ Add a task queue and job scheduling mechanism  
- 🚦 Implement traffic management for shared corridors and intersections  
- 🔄 Add failure detection with automatic task reassignment  
- 📊 Introduce performance metrics such as throughput and robot idle time  
- 🎥 Add RViz visualization for active robots, goals, and fleet state  
- 🤖 Integrate with Nav2 for full navigation stack support  

---

## 👤 Author

**Nived KRISHNAN**  
Robotics Engineer | AMR & Fleet Systems  

**Skills & Tools:**  
ROS 2 • C++ • Python • Autonomous Mobile Robots  

---
## 🙏 Credits

The Gazebo multi-robot simulation used in this project is based on the following repository:

- **tb3_multi_robot** by **Arshad Mehmood**  
  https://github.com/arshadlab/tb3_multi_robot.git  
---  
## ⭐ If You Like This Project

If this project helped or inspired you:

- ⭐ Star the repository  
- 🍴 Fork and extend it  
- 🧠 Use it as a base for your own fleet or research experiments  



