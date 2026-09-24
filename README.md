# Real-Time Reactive Teleoperation of a 7-DOF Robotic Arm

[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi-C51A4A.svg)](https://www.raspberrypi.com/)
[![Robot](https://img.shields.io/badge/Robot-Elephant%20Robotics%20MyArm%20300%20Pi-blue.svg)](https://www.elephantrobotics.com/)
[![Python](https://img.shields.io/badge/Python-3.9%2B-brightgreen.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A real-time, reactive teleoperation framework for the **Elephant Robotics MyArm 300 Pi** (7-DOF manipulator) equipped with a vacuum suction gripper, controlled via a standard gamepad/PS2 controller.

Developed for **Project 2: Robotic Systems I (2025–2026)**, Department of Electrical and Computer Engineering, University of Patras.

---

## 📌 Table of Contents
- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Control & Kinematics Formulation](#-control--kinematics-formulation)
  - [Forward Kinematics & Geometric Jacobian](#forward-kinematics--geometric-jacobian)
  - [Damped Least Squares (DLS) & Singularity Avoidance](#damped-least-squares-dls--singularity-avoidance)
  - [Redundancy Resolution & Null-Space Optimization](#redundancy-resolution--null-space-optimization)
- [Hardware & Safety Constraints](#-hardware--safety-constraints)
- [Repository Structure](#-repository-structure)
- [Wiring & Hardware Setup](#-wiring--hardware-setup)
- [Controller Mapping](#-controller-mapping)
- [Installation & Setup](#-installation--setup)
- [Running the System](#-running-the-system)
- [Contributors](#-contributors)

---

## 📖 Project Overview

The objective of this project is to provide intuitive, robust, and continuous Cartesian velocity teleoperation of the 7-DOF MyArm 300 Pi. Instead of controlling joints individually, an operator commands the linear and angular velocities of the end-effector in Cartesian space.

### Core Capabilities:
- **Low-Latency Feedback Loop:** Executes a closed control loop directly on the onboard Raspberry Pi at up to 100 Hz.
- **Continuous Reactive Tracking:** Converts analog joystick inputs into Cartesian velocity commands ($\dot{x} \in \mathbb{R}^6$).
- **Singularity Robustness:** Employs Yoshikawa manipulability-based Damped Least Squares (DLS) to guarantee stability near kinematic singularities.
- **Kinematic Redundancy Exploitation:** Solves the 7-DOF differential kinematics via null-space projection to keep joints near a preferred resting posture.
- **Fail-Safe Operation:** Features workspace clamping, joint limits clipping, command filtering, and dead-man stopping when joysticks are released.
- **Integrated Vacuum Gripper:** Hardware relay-based suction control (Grasp / Release) for pick-and-place and cube-stacking tasks.

---

## 🏗️ System Architecture

```
       ┌────────────────────────┐
       │   PS2 / USB Gamepad    │
       └───────────┬────────────┘
                   │ Axis Inputs & Button Events
                   ▼
       ┌────────────────────────┐
       │ controller_teleop.py   │ ◄── Deadzone filtering, input scaling,
       └───────────┬────────────┘     noise cutoffs & relay control
                   │ v_cmd = [vx, vy, vz, wx, wy, wz]^T
                   ▼
       ┌────────────────────────┐
       │   kinematics_new.py    │ ◄── FK, Geometric Jacobian (6x7),
       └───────────┬────────────┘     DLS Inverse Kinematics & Null-space projection
                   │ \dot{q} (Joint Velocities)
                   ▼
       ┌────────────────────────┐
       │        main.py         │ ◄── Joint limits, velocity clamping,
       └───────────┬────────────┘     and Cartesian workspace validation
                   │ q_next = q_current + \dot{q} * DT
                   ▼
       ┌────────────────────────┐
       │     MyArm 300 Pi       │ ◄── Serial transmission via /dev/ttyAMA0
       └────────────────────────┘
```

---

## 📐 Control & Kinematics Formulation

### Forward Kinematics & Geometric Jacobian
Forward kinematics is evaluated directly using homogeneous coordinate transformations derived from the robot's URDF model:

$$T_{0,e}(q) = \prod_{i=1}^{7} T_{i-1,i}(q_i)$$

The Geometric Jacobian $J(q) \in \mathbb{R}^{6 \times 7}$ maps joint rates to Cartesian end-effector velocities:

$$v = \begin{bmatrix} \dot{p} \\ \omega \end{bmatrix} = J(q) \dot{q}, \quad \text{where } J_i = \begin{bmatrix} z_{i-1} \times (p_e - p_{i-1}) \\ z_{i-1} \end{bmatrix}$$

### Damped Least Squares (DLS) & Singularity Avoidance
To prevent joint velocities from exploding near singularities, Yoshikawa's manipulability measure $w(q)$ is computed:

$$w(q) = \sqrt{\det(J(q) J(q)^T)}$$

When $w(q) < w_{\text{threshold}}$, an adaptive damping parameter $\lambda$ is activated:

$$\lambda = \lambda_{\max} \left(1 - \left(\frac{w}{w_{\text{threshold}}}\right)^2\right)$$

The regularized pseudoinverse is computed as:

$$J^{\dagger} = J^T (J J^T + \lambda^2 I_6)^{-1}$$

### Redundancy Resolution & Null-Space Optimization
Because the robot has 7 degrees of freedom ($n=7$) and the task operates in 6 Cartesian directions ($m=6$), kinematic redundancy is handled via orthogonal null-space projection:

$$\dot{q} = J^{\dagger} v_{\text{cmd}} + (I_7 - J^{\dagger} J) \dot{q}_{\text{null}}$$

where the secondary objective drives the robot toward a comfortable resting posture $q_{\text{rest}}$ without disturbing the primary Cartesian task:

$$\dot{q}_{\text{null}} = k_{\text{null}} (q_{\text{rest}} - q_{\text{current}})$$

---

## 🛡️ Hardware & Safety Constraints

1. **Velocity Saturation:** Maximum joint velocity is capped at $\dot{q}_{\max} = 0.8\text{ rad/s}$ via anti-jerk scaling.
2. **Joint Limits Clamping:**
   - $q_{\min} = [-160^\circ, -80^\circ, -175^\circ, -100^\circ, -165^\circ, -110^\circ, -165^\circ]$
   - $q_{\max} = [ 160^\circ,  80^\circ,  175^\circ,   80^\circ,  165^\circ,  110^\circ,  165^\circ]$
3. **Workspace Boundary Enclosure:**
   - $X \in [-0.35, 0.35]\text{ m}$
   - $Y \in [-0.35, 0.35]\text{ m}$
   - $Z \in [0.02, 0.45]\text{ m}$ (Enforces a $2\text{ cm}$ safety buffer above the work table).
4. **Deadband Filter:** Applied to all analog joystick axes to eliminate drift ($< 0.1$ threshold).

---

## 📁 Repository Structure

```text
.
├── controller_teleop.py   # PS2 Controller interface & GPIO vacuum relay management
├── kinematics_new.py      # URDF-based Forward Kinematics, Jacobian & DLS Solver
├── main.py                # Main real-time teleoperation loop & safety enforcement
├── test_arm.py            # Hardware communication test (Base joint movement)
├── test_axes.py           # Joystick axis diagnostic tool
├── test_gripper.py        # Vacuum suction relay logic test
├── requirements.txt       # Project dependencies
└── README.md              # Project documentation
```

---

## 🔌 Wiring & Hardware Setup

### Communication Interface
- **Bus:** Internal serial port on the Raspberry Pi
- **Device Port:** `/dev/ttyAMA0`
- **Baud Rate:** `115200`

### Suction Gripper Relay Pins (BCM Mode)
| GPIO Pin | Function | Grasp State (ON) | Release State (OFF) |
| :---: | :---: | :---: | :---: |
| **GPIO 20** | Control Pin 1 | `HIGH` | `LOW` |
| **GPIO 21** | Control Pin 2 | `LOW` | `HIGH` |

---

## 🎮 Controller Mapping

| Input | Axis / Button | Robot Action |
| :--- | :---: | :--- |
| **Left Stick X** | Axis 0 | End-Effector Forward / Backward ($V_x$) |
| **Left Stick Y** | Axis 1 | End-Effector Left / Right ($V_y$) |
| **LT / RT Triggers** | Axis 2 / 5 | End-Effector Up / Down ($V_z$) |
| **Right Stick X** | Axis 4 | Roll Rotation ($\omega_x$) |
| **Right Stick Y** | Axis 3 | Pitch Rotation ($\omega_y$) |
| **Button A** | Button 0 | **Grasp** (Activate Vacuum Pump) |
| **Button B** | Button 1 | **Release** (Deactivate Vacuum Pump) |
| **Button Start / X** | Button 7, 9, or 2 | **Home Reset** (Return to Safe Stance) |

---

## 🚀 Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/](https://github.com/)<your-username>/Reactive-Teleoperation-of-a-7-DOF-Robotic-Arm.git
   cd Reactive-Teleoperation-of-a-7-DOF-Robotic-Arm
   ```

2. **Set up Python Virtual Environment (recommended on Raspberry Pi):**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
   *(Ensure `pygame`, `numpy`, `pymycobot`, and `RPi.GPIO` are installed).*

4. **Set Serial Permissions:**
   ```bash
   sudo chmod 666 /dev/ttyAMA0
   ```

---

## ⚡ Running the System

### 1. Diagnostic Checks (First-Time Setup)
Test joystick axes:
```bash
python3 test_axes.py
```
Test serial link with MyArm 300 Pi:
```bash
python3 test_arm.py
```
Test vacuum gripper relay:
```bash
python3 test_gripper.py
```

### 2. Launching Teleoperation
Run the main control loop:
```bash
python3 main.py
```
- The robot will perform an **Auto-Reset** routine to a safe home pose (`Q_HOME`).
- Real-time end-effector coordinates and velocity streams will appear in the terminal.
- Press `Ctrl + C` at any time to safely shut down the control loop and reset the GPIO relay pins.

---

## 👥 Contributors

- **Student 1 Name** - ECE Dept., University of Patras
- **Student 2 Name** - ECE Dept., University of Patras

---
*Course: Robotic Systems I (2025–2026) | University of Patras*
