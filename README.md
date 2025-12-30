# ros2-panda-pick-and-place-vision
# ROS 2 Pick and Place with Vision (Franka Panda)

An autonomous **ROS 2 pick-and-place system** using **MoveIt 2**, **Gazebo**, and **OpenCV** to detect, grasp, and sort colored objects with a **Franka Emika Panda** robot.

This project demonstrates an end-to-end manipulation pipeline: **perception → TF → motion planning → control → simulation**.

---

## 🎯 Project Overview

The system performs autonomous pick-and-place based on object color:

1. A camera detects objects using OpenCV color segmentation
2. Object positions are transformed into the robot base frame using TF2
3. MoveIt 2 plans collision-aware grasp and place motions
4. ros2_control executes trajectories in Gazebo
5. Objects are placed at predefined locations based on detected color

The entire pipeline runs without manual intervention once launched.

---

## 🛠 Technologies Used

- **ROS 2 Humble**
- **MoveIt 2**
- **Gazebo**
- **OpenCV (Python)**
- **Franka Emika Panda**
- **ros2_control**
- **TF2**
- **RViz 2**

---

## 🧠 What I Learned

- Integrating perception with motion planning in ROS 2
- TF frame transformations for vision-based grasping
- MoveIt 2 planning pipelines and trajectory execution
- ros2_control configuration and controller debugging
- Synchronization issues between Gazebo, RViz, and MoveIt
- Structuring a complete ROS 2 workspace for reproducibility

---

## 📁 Repository Structure

```
ros2-panda-pick-and-place-vision/
├── src/
│   ├── panda_description/
│   ├── panda_moveit_config/
│   ├── panda_control/
│   ├── panda_vision/
│   └── panda_bringup/
├── demo/
│   ├── demo.gif
│   └── screenshots/
├── README.md
├── LICENSE
└── .gitignore
```

---

## 🚀 How to Run

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble installed
- Gazebo
- OpenCV

### 1. Clone the Repository

```bash
git clone https://github.com/oluomaoji/ros2-panda-pick-and-place-vision.git
cd ros2-panda-pick-and-place-vision
```

### 2. Build the Workspace

```bash
colcon build
source install/setup.bash
```

### 3. Launch the Simulation

```bash
ros2 launch panda_bringup pick_and_place.launch.py
```

### 4. Run the Vision Node

```bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G -p scan_duration:=5.0
```

The robot will automatically detect objects and perform pick-and-place actions based on color.

---

## 📸 Demo

![Pick and Place Demo](demo/demo.webm)

*(Additional screenshots available in the `demo/screenshots` folder)*

---

## 📌 Notes & Attribution

This project was developed following a structured ROS 2 manipulation masterclass. The system was implemented independently, with debugging, configuration, and extensions performed to gain a deep understanding of each component.

---

## 📜 License

This project is licensed under the MIT License.