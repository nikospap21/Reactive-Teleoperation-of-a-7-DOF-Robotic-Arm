import os
import sys
import time
import numpy as np

# Προσθήκη του root directory στο path
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)

from src.control.safety import SafetySupervisor
from src.input.ps2_controller import PS2Receiver
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver


class TeleoperationNode:

  def __init__(self, config_path="config/controller_config.yaml", rate_hz=30.0):
    self.dt = 1.0 / rate_hz
    self.rate_hz = rate_hz

    print("[*] Αρχικοποίηση Teleoperation Subsystems...")

    # 1. Hardware / Input interfaces
    self.ps2 = PS2Receiver(config_path=config_path)

    # 2. Kinematics & Redundancy solver
    self.kinematics = MyArmKinematics()
    self.solver = RedundancySolver(kinematics=self.kinematics)

    # 3. Safety Supervisor
    self.safety = SafetySupervisor(
        q_min=self.kinematics.joint_limits_lower,
        q_max=self.kinematics.joint_limits_upper,
        max_joint_vel=1.0,
        buffer_zone_deg=5.0,
    )

    # Αρχική διαμόρφωση (Home pose: ελαφρώς λυγισμένος για αποφυγή singularity)
    self.q_current = np.radians([0.0, 30.0, 0.0, 60.0, 0.0, 30.0, 0.0])
    self.is_running = False

  def run(self):
    self.is_running = True
    print(f"[*] Teleoperation Loop Started σε {self.rate_hz} Hz.")
    print("[*] Χρησιμοποίησε τα sticks για κίνηση. Πάτα Ctrl+C για έξοδο.\n")

    prev_gripper_state = False

    try:
      while self.is_running:
        loop_start_time = time.time()

        # --- 1. Ανάγνωση Χειριστηρίου ---
        vx, vy, vz, wz, gripper_on, has_data = self.ps2.update()

        # Σχηματισμός διανύσματος καρτεσιανής ταχύτητας (vx, vy, vz, wx, wy, wz)
        # Περιορίζουμε τις περιστροφές μόνο στο Yaw (wz) από το χειριστήριο
        v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

        # --- 2. Έλεγχος Gripper ---
        if gripper_on != prev_gripper_state:
          prev_gripper_state = gripper_on
          state_str = "ON (Suction Activated)" if gripper_on else "OFF (Release)"
          print(f"[GRIPPER] State changed: {state_str}")
          # Εδώ καλείται το hardware API της βεντούζας

        # --- 3. Forward Kinematics & Workspace Check ---
        ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
        v_safe_cartesian = self.safety.check_workspace(ee_pos, v_cartesian)

        # --- 4. Dead-man / Zero Velocity Check ---
        if np.allclose(v_safe_cartesian, 0.0):
          # Αν δεν υπάρχει εντολή, μηδενίζουμε τις ταχύτητες άμεσα
          q_dot_final = np.zeros(7)
        else:
          # --- 5. Redundancy Resolution (DLS + Null Space) ---
          q_dot_computed = self.solver.solve(self.q_current, v_safe_cartesian)

          # --- 6. Safety Clamping & Uniform Scaling ---
          q_dot_final = self.safety.filter_command(
              q_current=self.q_current, q_dot=q_dot_computed
          )

        # --- 7. Ενημέρωση Κατάστασης Ρομπότ (Forward Euler Integration) ---
        # Στο πραγματικό ρομπότ είτε στέλνουμε q_dot_final (Velocity control)
        # είτε q_next = q_current + q_dot_final * dt (Position control loop)
        self.q_current += q_dot_final * self.dt

        # Διασφάλιση αυστηρού clamping των γωνιών
        self.q_current = np.clip(
            self.q_current,
            self.kinematics.joint_limits_lower,
            self.kinematics.joint_limits_upper,
        )

        # Περιοδικό Telemetry (κάθε ~0.5 δευτερόλεπτο)
        if int(time.time() * 2) % 2 == 0 and not np.allclose(q_dot_final, 0.0):
          sys.stdout.write(
              f"\r[STATUS] EE Pos: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f},"
              f" {ee_pos[2]:.3f}] m | Max |q_dot|:"
              f" {np.max(np.abs(q_dot_final)):.3f} rad/s"
          )
          sys.stdout.flush()

        # --- 8. Διατήρηση Σταθερού Control Cycle (Loop Timing) ---
        elapsed = time.time() - loop_start_time
        sleep_time = self.dt - elapsed
        if sleep_time > 0:
          time.sleep(sleep_time)

    except KeyboardInterrupt:
      print("\n[*] Τερματισμός Teleoperation Loop από τον χρήστη.")
    finally:
      self.stop()

  def stop(self):
    print("[*] Ασφαλής ακινητοποίηση ρομπότ (Zero velocities).")
    # Εδώ αποστέλλεται εντολή μηδενικής ταχύτητας στο hardware


def main():
  node = TeleoperationNode(rate_hz=30.0)
  node.run()


if __name__ == "__main__":
  main()