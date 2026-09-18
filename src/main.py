import os
import sys
import time
import numpy as np

# Root path injection
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
  sys.path.insert(0, ROOT_DIR)

from src.control.robot_interface import MyArmInterface
from src.control.safety import SafetySupervisor
from src.gripper.vacuum_gripper import VacuumGripper
from src.input.ps2_controller import DirectPS2Controller
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver


class TeleoperationNode:

  def __init__(self, rate_hz=20.0):
    self.rate_hz = rate_hz
    self.dt = 1.0 / rate_hz

    print("[*] Αρχικοποίηση Teleoperation...")

    # 1. Επικοινωνία με τον πραγματικό MyArm 300 Pi
    self.robot = MyArmInterface()

    # 2. Χειριστήριο PS2
    self.ps2 = DirectPS2Controller()

    # 3. Βεντούζα Αναρρόφησης (Vacuum Gripper)
    self.gripper = VacuumGripper()

    # 4. Κινηματική, Solver & Safety
    self.kinematics = MyArmKinematics()
    self.solver = RedundancySolver(kinematics=self.kinematics)
    self.safety = SafetySupervisor(
        q_min=self.kinematics.joint_limits_lower,
        q_max=self.kinematics.joint_limits_upper,
        max_joint_vel=1.0,
        buffer_zone_deg=5.0,
    )

    # 5. Αρχική σταθεροποίηση σε ασφαλή/όρθια στάση
    print("[*] Μετάβαση στην αρχική θέση (Home Pose)...")
    self.home_pose_rad = np.radians([0.0, 20.0, 0.0, 50.0, 0.0, 30.0, 0.0])
    self.robot.send_joint_angles(self.home_pose_rad, speed=25)
    time.sleep(2.5)  # Χρόνος για να φτάσει και να σταθεροποιηθεί

    # Διάβασμα της πραγματικής θέσης εκκίνησης από τους encoders
    real_q = self.robot.get_joint_angles()
    if real_q is not None:
      self.q_current = np.array(real_q, dtype=float)
    else:
      self.q_current = np.copy(self.home_pose_rad)

    self.is_running = False

  def run(self):
    self.is_running = True
    print(f"[*] Control Loop ενεργό στα {self.rate_hz} Hz.")
    print("[*] Χειριστείτε τον βραχίονα. Πλήκτρο L1/R1 για βεντούζα.\n")

    prev_gripper_state = False

    try:
      while self.is_running:
        loop_start = time.time()

        # 1. Ανάγνωση PS2 controller
        vx, vy, vz, wz, gripper_req, connected = self.ps2.update()
        v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

        # 2. Χειρισμός Βεντούζας
        if gripper_req != prev_gripper_state:
          prev_gripper_state = gripper_req
          if gripper_req:
            self.gripper.activate()
          else:
            self.gripper.release()

        # 3. Dead-man Switch: Πλήρης ακινησία αν δεν υπάρχει είσοδος
        is_idle = np.all(np.abs(v_cartesian) < 1e-4)

        if is_idle:
          q_dot_final = np.zeros(7)
        else:
          # 4. Workspace safety
          ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
          v_safe = self.safety.check_workspace(ee_pos, v_cartesian)

          # 5. Differential IK & Redundancy Resolution
          q_dot_computed = self.solver.solve(self.q_current, v_safe)

          # 6. Safety clamping
          q_dot_final = self.safety.filter_command(
              self.q_current, q_dot_computed
          )

        # 7. Ολοκλήρωση θέσης & αποστολή στον πραγματικό βραχίονα
        self.q_current += q_dot_final * self.dt
        self.q_current = np.clip(
            self.q_current,
            self.kinematics.joint_limits_lower,
            self.kinematics.joint_limits_upper,
        )

        # ΜΟΝΟ ΑΝ υπάρχει κίνηση στέλνουμε εντολή θέσης/ταχύτητας
        if not is_idle:
          self.robot.send_joint_angles(self.q_current, speed=50)

        # 8. Τηλεμετρία
        ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
        sys.stdout.write(
            f"\r[JOYSTICK] vx:{vx:+.2f} vy:{vy:+.2f} vz:{vz:+.2f} wz:{wz:+.2f}"
            f" | EE: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}] m |"
            f" Grip: {'ON ' if gripper_req else 'OFF'}"
        )
        sys.stdout.flush()

        # Χρονισμός κύκλου
        elapsed = time.time() - loop_start
        if self.dt > elapsed:
          time.sleep(self.dt - elapsed)

    except KeyboardInterrupt:
      print("\n\n[*] Τερματισμός teleoperation...")
    finally:
      self.gripper.release()
      self.robot.stop()


def main():
  node = TeleoperationNode(rate_hz=20.0)
  node.run()


if __name__ == "__main__":
  main()