# Real-Time Reactive Teleoperation of a 7-DOF Robotic Arm

A real-time, reactive teleoperation framework for the **Elephant Robotics MyArm 300 Pi** (7-DOF manipulator) equipped with a vacuum suction gripper, driven via a standard USB gamepad / PS2-style controller.

Developed for **Project 2: Robotic Systems I (2025–2026)**, Department of Electrical and Computer Engineering, University of Patras.

## 📌 Table of Contents

* [Project Overview](#-project-overview)

* [System Architecture](#-system-architecture)

* [Control & Kinematics Formulation](#-control--kinematics-formulation)

  * [Forward Kinematics & Geometric Jacobian](#forward-kinematics--geometric-jacobian)

  * [Damped Least Squares (DLS) & Singularity Avoidance](#damped-least-squares-dls--singularity-avoidance)

  * [Redundancy Resolution & Null-Space Optimization](#redundancy-resolution--null-space-optimization)

* [Hardware & Safety Constraints](#-hardware--safety-constraints)

* [Repository Structure](#-repository-structure)

* [Wiring & Hardware Setup](#-wiring--hardware-setup)

* [Controller Mapping](#-controller-mapping)

* [Installation & Setup](#-installation--setup)

* [Running the System](#-running-the-system)

* [Contributors](#-contributors)

## 📖 Project Overview

The objective of this project is to provide continuous, reactive Cartesian velocity teleoperation for the 7-DOF MyArm 300 Pi. Instead of commanding individual joints independently, the operator directly dictates the linear and angular velocities of the end-effector in Cartesian space $(\mathbb{R}^3 \times SO(3))$ using analog gamepad inputs.

### Core Capabilities:

* **Real-Time Control Loop:** Executes a closed differential control loop locally on the onboard Raspberry Pi at **30 Hz** ($\Delta t \approx 0.033\text{ s}$).

* **Continuous Reactive Tracking:** Transforms raw joystick and trigger inputs into 6D Cartesian spatial velocity vectors ($v_{\text{cmd}} \in \mathbb{R}^6$).

* **Direct URDF Kinematics:** Bypasses classical DH limitations by extracting joint frames, link offsets, and roll-pitch-yaw transforms directly from the official robot URDF.

* **Singularity Robustness:** Employs Yoshikawa manipulability-based Damped Least Squares (DLS) inverse kinematics to guarantee numerical stability near singular configurations.

* **Kinematic Redundancy Exploitation:** Exploits the extra degree of freedom ($7 - 6 = 1$) via orthogonal null-space projection, smoothly pulling the arm toward a safe resting posture ($q_{\text{rest}}$).

* **Tool-Oriented Pure Yaw Override:** Direct coupling of the 7th joint when pure yaw ($\omega_z$) rotation is commanded to prevent unneeded base arm reconfiguration.

* **Integrated Vacuum Gripper:** Hardware relay module control (Grasp / Release) over GPIO pins for pick-and-place and cube-stacking tasks.

## 🏗️ System Architecture

```
       ┌────────────────────────┐
       │   PS2 / USB Gamepad    │
       └───────────┬────────────┘
                   │ Axis Inputs & Button Events
                   ▼
       ┌────────────────────────┐
       │  controller_teleop.py  │ ◄── Deadzone filtering, trigger normalization,
       └───────────┬────────────┘     noise cutoffs & GPIO relay switching
                   │ v_raw = [vx, vy, vz, wx, wy, wz]^T
                   ▼
       ┌────────────────────────┐
       │     kinematics.py      │ ◄── Direct URDF FK, Geometric Jacobian (6x7),
       └───────────┬────────────┘     DLS Inverse Kinematics & Null-Space Projection
                   │ \dot{q} (Joint Velocities)
                   ▼
       ┌────────────────────────┐
       │        main.py         │ ◄── Frame transformation (Tool -> Base),
       └───────────┬────────────┘     workspace & joint limits verification
                   │ q_next = q_current + \dot{q} * DT
                   ▼
       ┌────────────────────────┐
       │      MyArm 300 Pi      │ ◄── Serial communication via /dev/ttyAMA0 (115200 baud)
       └────────────────────────┘

```

## 📐 Control & Kinematics Formulation

### Forward Kinematics & Geometric Jacobian

Forward kinematics is evaluated using homogeneous transformation matrices derived from the physical URDF specifications:

$$
T_{0,e}(q) = \prod_{i=0}^{6} T_i(q_i) = \begin{bmatrix} R_{\text{end}} & p_{\text{end}} \\ 0^T & 1 \end{bmatrix}
$$

Each transformation incorporates the link offsets (such as $y = -0.110\text{ m}$ between Joint 2–3 and $y = -0.126\text{ m}$ between Joint 4–5), the end-effector offset ($EE\_Offset = 0.15\text{ m}$ along the Z-axis of Joint 7), and joint rotations around the local Z-axis.

The Geometric Jacobian $J(q) \in \mathbb{R}^{6 \times 7}$ maps joint rates $\dot{q}$ to end-effector Cartesian velocities:

$$
v_{\text{cmd}} = \begin{bmatrix} v_{\text{lin}} \\ \omega \end{bmatrix} = J(q) \dot{q}, \quad \text{where } J_i = \begin{bmatrix} z_i \times (p_{\text{end}} - p_i) \\ z_i \end{bmatrix}
$$

### Damped Least Squares (DLS) & Singularity Avoidance

To prevent joint velocities from diverging near singular configurations, Yoshikawa's manipulability measure $w(q)$ is computed:

$$
w(q) = \sqrt{\max\left(0, \det(J(q) J(q)^T)\right)}
$$

When $w(q) < w_{\text{threshold}} = 0.015$, dynamic damping $\lambda$ is activated:

$$
\lambda = \lambda_{\max} \left(1 - \left(\frac{w}{w_{\text{threshold}}}\right)^2\right), \quad \text{with } \lambda_{\max} = 0.04
$$

The regularized pseudoinverse is computed as:

$$
J_{\text{DLS}}^{\dagger} = J^T (J J^T + \lambda^2 I_6)^{-1}
$$

### Redundancy Resolution & Null-Space Optimization

Given that the arm possesses 7 DOFs and Cartesian tasks define 6 DOFs, the remaining degree of freedom is optimized within the null space of the Jacobian:

$$
\dot{q} = J_{\text{DLS}}^{\dagger} v_{\text{cmd}} + (I_7 - J_{\text{DLS}}^{\dagger} J) \dot{q}_{\text{null}}
$$

The secondary objective smoothly pulls the configuration toward the rest posture $q_{\text{rest}} = [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]^T\text{ rad}$:

$$
\dot{q}_{\text{null}} = k_{\text{null}} (q_{\text{rest}} - q_{\text{current}}), \quad \text{with } k_{\text{null}} = 0.1
$$

If any joint rate exceeds $\dot{q}_{\max} = 1.5\text{ rad/s}$, the vector $\dot{q}$ is uniformly scaled to preserve directional fidelity.

## 🛡️ Hardware & Safety Constraints

1. **Velocity Saturation:** Scaled dynamically to maintain maximum joint speed $\le 1.5\text{ rad/s}$.

2. **Joint Mechanical Limits (in radians):**

   * $q_{\min} = [-2.7925, -1.2217, -2.9670, -1.9722, -2.9670, -2.0071, -3.14159]$

   * $q_{\max} = [ 2.7925,  2.0071,  2.9670,  1.3089,  2.9670,  2.0071,  3.14159]$

3. **Cartesian Workspace Limits (with** $EE\_Offset = 0.15\text{ m}$**):**

   * $X \in [-0.53, 0.53]\text{ m}$

   * $Y \in [-0.53, 0.53]\text{ m}$

   * $Z \in [-0.08, 0.53]\text{ m}$ (The lower limit allows reliable suction contact with tabletop objects).

4. **Trigger Normalization & Deadzone Filtering:** Analog triggers are normalized from $[-1, 1]$ to $[0, 1]$. Deadband thresholding and velocity cutoffs ($< 0.02\text{ m/s}$ and $< 0.02\text{ rad/s}$) ensure strict immobility when controls are idle.

## 📁 Repository Structure

```
.
├── controller_teleop.py   # PS2/Gamepad input processing & GPIO relay driver
├── kinematics.py          # Direct URDF Forward Kinematics, Jacobian & DLS solver
├── main.py                # Main 30 Hz closed-loop teleoperation and safety supervisor
├── test_axes.py           # Real-time diagnostic tool for joystick axes & triggers
├── test_gripper.py        # Diagnostic script verifying GPIO relay states (Grasp/Release)
├── robotics_report.pdf    # Full technical project report
└── README.md              # Project documentation

```

## 🔌 Wiring & Hardware Setup

### Serial Communication

* **Interface:** Internal Raspberry Pi UART bus

* **Device Port:** `/dev/ttyAMA0`

* **Baud Rate:** `115200`

### Suction Gripper Relay (BCM Pinout)

The vacuum pump is powered via an external power supply with a shared ground to the Raspberry Pi, switched through a 2-channel 5V relay module:

| 

| **Relay Input** | **GPIO Pin (BCM)** | **Grasp State (Suction ON)** | **Release State (Suction OFF)** | 
| **IN1** | **GPIO 20** | `HIGH` | `LOW` | 
| **IN2** | **GPIO 21** | `LOW` | `HIGH` | 

## 🎮 Controller Mapping

| **Controller Input** | **Pygame Axis / Button** | **Robot Action** | **Scaling Factor** | 
| **Left Stick X** | Axis 0 | Longitudinal Translation $V_x$ (Forward / Backward) | $0.05\text{ m/s}$ | 
| **Left Stick Y** | Axis 1 | Lateral Translation $V_y$ (Left / Right) | $0.05\text{ m/s}$ | 
| **Triggers (RT - LT)** | Axis 5 - Axis 2 | Vertical Translation $V_z$ (Up / Down) | $0.05\text{ m/s}$ | 
| **Right Stick X** | Axis 4 | Roll Angular Velocity $\omega_x$ | $0.05\text{ rad/s}$ | 
| **Right Stick Y** | Axis 3 | Pitch Angular Velocity $\omega_y$ | $0.05\text{ rad/s}$ | 
| **Bumpers (RB - LB)** | Button 5 - Button 4 | Yaw Angular Velocity $\omega_z$ | $0.05\text{ rad/s}$ | 
| **Button A** | Button 0 | **Grasp** (Activate Vacuum Pump) | Digital Toggle | 
| **Button B** | Button 1 | **Release** (Deactivate Vacuum Pump) | Digital Toggle | 
| **Button X** | Button 2 | **Home Reset** (Return to $Q_{\text{HOME}}$) | Event Trigger | 

## 🚀 Installation & Setup

1. **Clone or transfer the project directory onto the Raspberry Pi:**

   ```
   cd ~/Reactive-Teleoperation-of-a-7-DOF-Robotic-Arm
   
   ```

2. **Install required system and Python dependencies:**

   ```
   sudo apt-get update
   sudo apt-get install python3-pygame python3-rpi.gpio
   pip3 install pymycobot numpy
   
   ```

3. **Configure device permissions for UART and Gamepad:**

   ```
   sudo chmod 666 /dev/ttyAMA0
   sudo chmod 666 /dev/input/js0
   
   ```

## ⚡ Running the System

### 1. Hardware Verification (Optional Diagnostics)

Verify controller axis mapping and calibration:

```
python3 test_axes.py

```

Cycle through relay test combinations to verify suction pump switching:

```
python3 test_gripper.py

```

### 2. Main Teleoperation

Start the real-time teleoperation supervisor:

```
python3 main.py

```

* The robot automatically powers on, syncs current joint state, and transitions smoothly to the home posture ($Q_{\text{HOME}}$).

* The console displays real-time Cartesian end-effector coordinates $[X, Y, Z]$ and joint angle vectors $[q_0, \dots, q_6]$.

* Pressing **Button X** triggers an immediate reset to $Q_{\text{HOME}}$.

* Press `Ctrl + C` at any time to safely shut down the control loop, disable the vacuum pump, and clean up the GPIO states.

## 👥 Contributors

* **Nikolaos Papoutsopoulos** (AM: 1101000) – Department of ECE, University of Patras

* **Marios Christos Choustoulakis** (AM: 1101141) – Department of ECE, University of Patras

*Course: Robotic Systems I (2025–2026) | Department of Electrical and Computer Engineering, University of Patras*