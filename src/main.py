import os
import sys
import time
import numpy as np

# Root path injection
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
  sys.path.insert(0, ROOT_DIR)

from src.control.safety import SafetySupervisor
from src.input.ps2_controller import DirectPS2Controller
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver


class TeleoperationNode:

  def __init__(self, rate_hz=30.0):
    self.dt = 1.0 / rate_hz
    self.rate_hz = rate_hz

    print("[*] Αρχικοποίηση Teleoperation...")

    # 1. Απευθείας USB σύνδεση χειριστηρίου
    self.ps2 = DirectPS2Controller()

    # 2. Κινηματική & Redundancy solver
    self.kinematics = MyArmKinematics()
    self.solver = RedundancySolver(kinematics=self.kinematics)

    # 3. Safety Supervisor
    self.safety = SafetySupervisor(
        q_min=self.kinematics.joint_limits_lower,
        q_max=self.kinematics.joint_limits_upper,
        max_joint_vel=1.0,
        buffer_zone_deg=5.0,
    )

    # Αρχική ασφαλής θέση (Home pose σε ακτίνια)
    self.q_current = np.radians([0.0, 30.0, 0.0, 60.0, 0.0, 30.0, 0.0])
    self.is_running = False

  def run(self):
    self.is_running = True
    print(f"[*] Loop ξεκίνησε στα {self.rate_hz} Hz.")
    print("[*] Κούνα τα sticks. Πάτα Ctrl+C για έξοδο.\n")

    prev_gripper = False

    try:
      while self.is_running:
        loop_start = time.time()

        # 1. Ανάγνωση απευθείας από το USB gamepad
        vx, vy, vz, wz, gripper, connected = self.ps2.update()

        # Καρτεσιανό διάνυσμα ταχύτητας [vx, vy, vz, wx, wy, wz]
        v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

        # 2. Έλεγχος Gripper
        if gripper != prev_gripper:
          prev_gripper = gripper
          print(f"\n[GRIPPER] {'ON' if gripper else 'OFF'}")

        # 3. Dead-man Switch: Αν όλα τα inputs είναι ~0, πλήρης στάση
        # Αυτό αποτρέπει την αυτόνομη κίνηση του null space drift
        is_idle = np.all(np.abs(v_cartesian) < 1e-4)

        if is_idle:
          q_dot_final = np.zeros(7)
        else:
          # 4. Workspace safety check
          ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
          v_safe = self.safety.check_workspace(ee_pos, v_cartesian)

          # 5. Επίλυση διαφορικής κινηματικής & Redundancy
          q_dot_computed = self.solver.solve(self.q_current, v_safe)

          # 6. Safety clamping σε ταχύτητες και όρια
          q_dot_final = self.safety.filter_command(
              self.q_current, q_dot_computed
          )

        # 7. Ολοκλήρωση θέσης
        self.q_current += q_dot_final * self.dt
        self.q_current = np.clip(
            self.q_current,
            self.kinematics.joint_limits_lower,
            self.kinematics.joint_limits_upper,
        )

        # 8. Real-time Telemetry (εμφάνιση ταχυτήτων και θέσης)
        ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
        sys.stdout.write(
            f"\r[JOYSTICK] vx:{vx:+.2f} vy:{vy:+.2f} vz:{vz:+.2f} wz:{wz:+.2f}"
            f" | EE: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}] m |"
            f" Gripper: {'ON ' if gripper else 'OFF'}"
        )
        sys.stdout.flush()

        # Loop rate sleep
        elapsed = time.time() - loop_start
        if self.dt > elapsed:
          time.sleep(self.dt - elapsed)

    except KeyboardInterrupt:
      print("\n\n[*] Τερματισμός teleoperation.")


def main():
  node = TeleoperationNode(rate_hz=30.0)
  node.run()


if __name__ == "__main__":
  main()