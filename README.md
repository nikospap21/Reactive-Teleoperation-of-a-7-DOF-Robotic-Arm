# Reactive Teleoperation of a 7-DOF Robotic Arm

Real-time reactive teleoperation framework for the **Elephant Robotics MyArm 300 Pi** (7-DOF robotic manipulator) equipped with a vacuum suction gripper, controlled via a PS2 joystick.

Developed as part of the **Robotic Systems I** course (2025–2026), Department of Electrical and Computer Engineering, University of Patras.

---

## 📌 Project Overview

The objective is to enable an operator to control the robot end-effector directly in Cartesian space (translation and rotation) rather than joint-by-joint. The system processes joystick inputs in a continuous real-time loop, resolves kinematics while handling redundancy, and ensures safe execution under workspace and joint constraints.

### Key Features
- **Real-Time Joystick Mapping:** Continuous Cartesian translation ($x, y, z$) and orientation control via PS2 controller inputs.
- **Differential Kinematics & Velocity Control:** Jacobian-based mapping from Cartesian velocities to joint velocities.
- **Redundancy Resolution:** Exploitation of null-space motion for joint-limit avoidance and posture optimization.
- **Robust Constraint Handling:** Safe rejection/modification of infeasible commands (singularities, joint limits, workspace boundaries) without blocking or deadlocks.
- **End-Effector Manipulation:** Gripper activation and release logic for pick-and-place and stacking tasks.

---

## 🛠️ System Architecture

```
[ PS2 Controller ] 
       │ (raw inputs)
       ▼
[ Input Preprocessing & Deadzone Filtering ]
       │ (Cartesian velocities v_e)
       ▼
[ Redundancy Resolution & Differential IK (Jacobian) ]
       │ (Joint velocities \dot{q})
       ▼
[ Safety, Joint-Limit & Singularity Verification ]
       │ (Safe \dot{q})
       ▼
[ MyArm 300 Pi Controller ]
```

---

## 🚀 Getting Started

### Prerequisites
- Python 3.8+
- Robot communication library (`pymycobot` or ROS 2 driver)
- Joystick driver (`pygame` or `inputs`)
- NumPy / SciPy

### Installation
```bash
git clone [https://github.com/](https://github.com/)<username>/myarm-300-pi-teleop.git
cd myarm-300-pi-teleop
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Running the Teleoperation Node
```bash
python main.py
```

---

## 🎮 Controller Mapping

| Button / Stick | Action |
| :--- | :--- |
| **Left Stick (X/Y)** | End-effector Translation ($X, Y$) |
| **Right Stick (Y)** | End-effector Translation ($Z$ - Up/Down) |
| **Right Stick (X)** | End-effector Yaw Rotation |
| **L1 / R1** | Gripper Vacuum ON / OFF |
| **Select / Stop** | Emergency Stop / Safe Halt |

---

## 👥 Contributors
- **Nikolaos Papoutsopoulos** - up1101000
- **Marios Choustoulakhs** - up1101141
