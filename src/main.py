import os
import sys
import time
import numpy as np

# Προσθήκη του root directory στο Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.input.ps2_controller import PS2Receiver
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver
from src.control.safety import SafetySupervisor
from src.control.robot_interface import MyArmInterface
from src.gripper.vacuum_gripper import VacuumGripper
from src.visualization import RVizVisualizer


class TeleoperationNode:
    def __init__(
        self,
        config_path="config/controller_config.yaml",
        rate_hz=30.0,
        serial_port="/dev/ttyAMA0",
        mock_robot=True
    ):
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        print("=" * 65)
        print("   Elephant Robotics MyArm 300 Pi - Integrated Teleop Node   ")
        print("=" * 65)

        # 1. PS2 Receiver (UDP Socket Listener)
        print("[*] Αρχικοποίηση PS2 Controller Receiver...")
        self.ps2 = PS2Receiver(config_path=config_path)

        # 2. Κινηματικό Μοντέλο & Redundancy Solver
        print("[*] Φόρτωση Κινηματικής & Redundancy Solver...")
        self.kinematics = MyArmKinematics()
        self.solver = RedundancySolver(kinematics=self.kinematics)

        # 3. Safety Supervisor (Joint limits, Singularity damping, Workspace)
        print("[*] Εκκίνηση Safety Supervisor...")
        self.safety = SafetySupervisor(
            q_min=self.kinematics.joint_limits_lower,
            q_max=self.kinematics.joint_limits_upper,
            max_joint_vel=1.0,      # rad/s μέγιστη επιτρεπόμενη ταχύτητα
            buffer_zone_deg=5.0      # Ζώνη προστασίας ορίων (5 μοίρες)
        )

        # 4. Robot Hardware Interface
        print(f"[*] Σύνδεση με MyArm Interface (Port: {serial_port}, Mock: {mock_robot})...")
        self.robot = MyArmInterface(
            port=serial_port,
            baudrate=115200,
            mock=mock_robot,
            joint_limits_lower=self.kinematics.joint_limits_lower,
            joint_limits_upper=self.kinematics.joint_limits_upper
        )

        # 5. Vacuum Suction Gripper Interface
        print("[*] Αρχικοποίηση Vacuum Gripper...")
        self.gripper = VacuumGripper(
            arm_interface=self.robot,
            pin_pump=20,
            pin_valve=21,
            use_gpio=False,          # True αν συνδεθεί απευθείας στα pins του Pi
            mock=mock_robot
        )

        # 6. ROS 2 RViz Visualizer (Publishing στο /joint_states)
        print("[*] Αρχικοποίηση RViz Visualizer...")
        self.visualizer = RVizVisualizer(node_name="myarm_teleop_visualizer")

        # Αρχικοποίηση σε Home Pose
        self.robot.go_home(speed=30)
        self.q_current = self.robot.get_joint_angles()

        self.prev_gripper_state = False
        self.is_running = False

    def run(self):
        self.is_running = True
        print("\n[+] Το σύστημα είναι πλήρως ενεργό!")
        print("[+] Αριστερό Stick: Μετατόπιση X / Y")
        print("[+] Δεξί Stick: Ύψος Z & Περιστροφή Yaw (wz)")
        print("[+] Κουμπιά A / B: Suction Gripper ON / OFF")
        print("[+] Πάτα Ctrl+C για ασφαλή έξοδο.\n")

        try:
            while self.is_running:
                loop_start = time.time()

                # --- 1. Ανάγνωση δεδομένων από το χειριστήριο ---
                vx, vy, vz, wz, gripper_on, has_data = self.ps2.update()

                # Διάνυσμα επιθυμητής καρτεσιανής ταχύτητας: [vx, vy, vz, wx, wy, wz]
                v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

                # --- 2. Διαχείριση βεντούζας αναρρόφησης (Vacuum Gripper) ---
                if gripper_on != self.prev_gripper_state:
                    self.prev_gripper_state = gripper_on
                    self.gripper.set_state(gripper_on)

                # --- 3. Ανάγνωση τρέχουσας διαμόρφωσης αρθρώσεων ---
                self.q_current = self.robot.get_joint_angles()

                # --- 4. Forward Kinematics & Workspace Boundaries Check ---
                ee_pos, _ = self.kinematics.get_end_effector_pose(self.q_current)
                v_safe_cartesian = self.safety.check_workspace(ee_pos, v_cartesian)

                # --- 5. Επίλυση Διαφορικής Κινηματικής & Null-Space Redundancy ---
                if np.allclose(v_safe_cartesian, 0.0):
                    # Dead-man stop: Μηδενισμός ταχυτήτων αν δεν δίνεται εντολή
                    q_dot_final = np.zeros(7)
                else:
                    # Damped Least Squares Inverse & Null-space posture projection
                    q_dot_computed = self.solver.solve(self.q_current, v_safe_cartesian)

                    # Directional joint clamping & safety scaling
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

                # --- 7. Αποστολή στο RViz (/joint_states) ---
                self.visualizer.publish_joints(self.q_current)

                # --- 8. Live Terminal Telemetry ---
                max_vel = np.max(np.abs(q_dot_final))
                gripper_status = "[ON]" if self.gripper.is_active else "[OFF]"
                sys.stdout.write(
                    f"\r[STATUS] EE: X:{ee_pos[0]:+.3f} Y:{ee_pos[1]:+.3f} Z:{ee_pos[2]:+.3f} m | "
                    f"Max |q_dot|: {max_vel:.2f} rad/s | Gripper: {gripper_status}"
                )
                sys.stdout.flush()

                # --- 9. Διατήρηση συχνότητας βρόχου ---
                elapsed = time.time() - loop_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n[*] Διακοπή λειτουργίας από τον χρήστη (Ctrl+C).")
        finally:
            self.shutdown()

    def shutdown(self):
        print("[*] Τερματισμός συστήματος...")
        self.gripper.cleanup()
        self.robot.stop_motion()
        self.visualizer.shutdown()
        print("[+] Όλα τα υποσυστήματα ακινητοποιήθηκαν με ασφάλεια.")


def main():
    # mock_robot=True για offline εκτέλεση και δοκιμές χωρίς φυσικό υλικό
    # mock_robot=False όταν ο κώδικας εκτελείται στο MyArm 300 Pi του εργαστηρίου
    node = TeleoperationNode(
        config_path="config/controller_config.yaml",
        rate_hz=30.0,
        serial_port="/dev/ttyAMA0",
        mock_robot=True
    )
    node.run()


if __name__ == "__main__":
    main()