import os
import sys
import time
import numpy as np

# Προσθήκη του root directory στο path ώστε να φορτώνονται τα modules του src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.input.ps2_controller import PS2Receiver
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver
from src.control.safety import SafetySupervisor
from src.control.robot_interface import MyArmInterface
from src.gripper.vacuum_gripper import VacuumGripper


class TeleoperationNode:
    def __init__(
        self,
        config_path="config/controller_config.yaml",
        rate_hz=30.0,
        serial_port="/dev/ttyAMA0",
        mock_robot=False
    ):
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        print("=" * 60)
        print("   Elephant Robotics MyArm 300 Pi - Teleoperation Node   ")
        print("=" * 60)

        # 1. Receiver Χειριστηρίου PS2 (UDP listener)
        print("[*] Αρχικοποίηση PS2 Receiver...")
        self.ps2 = PS2Receiver(config_path=config_path)

        # 2. Κινηματικό Μοντέλο & Redundancy Solver
        print("[*] Φόρτωση Κινηματικής & Redundancy Solver...")
        self.kinematics = MyArmKinematics()
        self.solver = RedundancySolver(kinematics=self.kinematics)

        # 3. Safety Supervisor (Όρια αρθρώσεων, ταχύτητας & workspace)
        print("[*] Εκκίνηση Safety Supervisor...")
        self.safety = SafetySupervisor(
            q_min=self.kinematics.joint_limits_lower,
            q_max=self.kinematics.joint_limits_upper,
            max_joint_vel=1.0,      # rad/s μέγιστη ταχύτητα άρθρωσης
            buffer_zone_deg=5.0      # Buffer αποφυγής ορίων (5 μοίρες)
        )

        # 4. Robot Hardware Interface
        print(f"[*] Σύνδεση με ρομπότ (Port: {serial_port}, Mock: {mock_robot})...")
        self.robot = MyArmInterface(
            port=serial_port,
            baudrate=115200,
            mock=mock_robot,
            joint_limits_lower=self.kinematics.joint_limits_lower,
            joint_limits_upper=self.kinematics.joint_limits_upper
        )

        # 5. Vacuum Suction Gripper
        print("[*] Αρχικοποίηση Vacuum Gripper...")
        self.gripper = VacuumGripper(
            arm_interface=self.robot,
            pin_pump=20,
            pin_valve=21,
            use_gpio=False,          # True αν συνδέεται απευθείας στα pins του Pi
            mock=mock_robot
        )

        # Αρχικοποίηση θέσης (Home Pose)
        self.robot.go_home(speed=30)
        self.q_current = self.robot.get_joint_angles()

        self.prev_gripper_state = False
        self.is_running = False

    def run(self):
        self.is_running = True
        print("\n[+] Το σύστημα είναι ενεργό και έτοιμο!")
        print("[+] Αριστερός μοχλός: Μετατόπιση X / Y")
        print("[+] Δεξιός μοχλός: Ύψος Z & Περιστροφή Yaw (wz)")
        print("[+] Κουμπιά A / B: Ενεργοποίηση / Απελευθέρωση Suction Gripper")
        print("[+] Πάτα Ctrl+C για ασφαλή έξοδο.\n")

        try:
            while self.is_running:
                loop_start = time.time()

                # --- 1. Ανάγνωση δεδομένων από το PS2 ---
                vx, vy, vz, wz, gripper_on, has_data = self.ps2.update()

                # Διάνυσμα επιθυμητής καρτεσιανής ταχύτητας: [vx, vy, vz, wx, wy, wz]
                v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

                # --- 2. Έλεγχος Gripper (Vacuum Suction) ---
                if gripper_on != self.prev_gripper_state:
                    self.prev_gripper_state = gripper_on
                    self.gripper.set_state(gripper_on)

                # --- 3. Ανάγνωση τρέχουσας γωνιακής θέσης ρομπότ ---
                self.q_current = self.robot.get_joint_angles()

                # --- 4. Forward Kinematics & Workspace Check ---
                ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
                v_safe_cartesian = self.safety.check_workspace(ee_pos, v_cartesian)

                # --- 5. Επίλυση διαφορικής κινηματικής & Redundancy ---
                if np.allclose(v_safe_cartesian, 0.0):
                    # Dead-man stop: Αν δεν υπάρχει εντολή από μοχλό, σταματάμε άμεσα
                    q_dot_final = np.zeros(7)
                else:
                    # DLS Inverse & Null-space gradient projection
                    q_dot_computed = self.solver.solve(self.q_current, v_safe_cartesian)

                    # Directional joint clamping & uniform velocity scaling
                    q_dot_final = self.safety.filter_command(
                        q_current=self.q_current,
                        q_dot=q_dot_computed
                    )

                # --- 6. Αποστολή εντολών στο ρομπότ ---
                if np.allclose(q_dot_final, 0.0):
                    self.robot.stop_motion()
                else:
                    self.robot.send_joint_velocities(
                        q_dot=q_dot_final,
                        q_current=self.q_current,
                        dt=self.dt,
                        speed=90
                    )

                # --- 7. Live Telemetry στο Terminal ---
                max_vel = np.max(np.abs(q_dot_final))
                gripper_str = "[ON]" if self.gripper.is_active else "[OFF]"
                sys.stdout.write(
                    f"\r[STATUS] EE: X:{ee_pos[0]:+.3f} Y:{ee_pos[1]:+.3f} Z:{ee_pos[2]:+.3f} m | "
                    f"Max |q_dot|: {max_vel:.2f} rad/s | Gripper: {gripper_str}"
                )
                sys.stdout.flush()

                # --- 8. Διατήρηση σταθερής συχνότητας βρόχου ---
                elapsed = time.time() - loop_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n[*] Διακοπή λειτουργίας από τον χρήστη (Ctrl+C).")
        finally:
            self.shutdown()

    def shutdown(self):
        print("[*] Τερματισμός... Ακινητοποίηση βραχίονα και απελευθέρωση gripper.")
        self.gripper.cleanup()
        self.robot.stop_motion()
        print("[+] Το σύστημα τέθηκε εκτός λειτουργίας με ασφάλεια.")


def main():
    # Θέσε mock_robot=True για δοκιμή χωρίς το ρομπότ, ή False στο Raspberry Pi του εργαστηρίου
    node = TeleoperationNode(
        config_path="config/controller_config.yaml",
        rate_hz=30.0,
        serial_port="/dev/ttyAMA0",
        mock_robot=False
    )
    node.run()


if __name__ == "__main__":
    main()