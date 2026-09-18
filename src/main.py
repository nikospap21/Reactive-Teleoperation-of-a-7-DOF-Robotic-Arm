import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.input.ps2_controller import DirectPS2Controller
from src.kinematics.model import MyArmKinematics
from src.kinematics.redundancy_solver import RedundancySolver
from src.control.safety import SafetySupervisor
from src.control.robot_interface import MyArmInterface
from src.gripper.vacuum_gripper import VacuumGripper


def run_teleoperation():
    RATE_HZ = 30.0
    DT = 1.0 / RATE_HZ

    print("=" * 60)
    print("   MyArm 300 Pi - Standalone Teleoperation on Raspberry Pi   ")
    print("=" * 60)

    # 1. Hardware Interfaces
    ps2 = DirectPS2Controller(config_path="config/controller_config.yaml")
    kinematics = MyArmKinematics()
    solver = RedundancySolver(kinematics=kinematics)

    safety = SafetySupervisor(
        q_min=kinematics.joint_limits_lower,
        q_max=kinematics.joint_limits_upper,
        max_joint_vel=1.0,
        buffer_zone_deg=6.0
    )

    # Αν είστε στο πραγματικό ρομπότ, βάλτε mock=False και τη σωστή θύρα (/dev/ttyAMA0 ή /dev/ttyUSB0)
    robot = MyArmInterface(
        port="/dev/ttyAMA0",
        baudrate=115200,
        mock=False,
        joint_limits_lower=kinematics.joint_limits_lower,
        joint_limits_upper=kinematics.joint_limits_upper
    )

    gripper = VacuumGripper(arm_interface=robot, mock=False)

    print("[*] Μετάβαση σε Home Pose...")
    robot.go_home(speed=25)
    prev_gripper = False

    print("\n[+] Σύστημα έτοιμο. Οδηγίες:")
    print(" - Αριστερός μοχλός: X / Y")
    print(" - Δεξί μοχλός: Z (ύψος) & Yaw (περιστροφή)")
    print(" - Κουμπί 0 / 1: Suction Gripper ON / OFF")
    print(" - Πάτα Ctrl+C για έξοδο.\n")

    try:
        while True:
            t_start = time.time()

            # 1. Ανάγνωση Χειριστηρίου
            vx, vy, vz, wz, gripper_on, connected = ps2.update()
            v_cartesian = np.array([vx, vy, vz, 0.0, 0.0, wz], dtype=float)

            # 2. Έλεγχος Βεντούζας
            if gripper_on != prev_gripper:
                prev_gripper = gripper_on
                gripper.set_state(gripper_on)

            # 3. Ανάγνωση Κατάστασης Ρομπότ
            q_current = robot.get_joint_angles()

            # 4. Workspace & Safety Checks
            ee_pos, _ = kinematics.get_end_effector_pose(q_current)
            v_safe = safety.check_workspace(ee_pos, v_cartesian)

            # 5. Differential Kinematics & Redundancy
            if np.allclose(v_safe, 0.0):
                q_dot_final = np.zeros(7)
            else:
                q_dot_raw = solver.solve(q_current, v_safe)
                q_dot_final = safety.filter_command(q_current, q_dot_raw)

            # 6. Αποστολή στο Υλικό
            if np.allclose(q_dot_final, 0.0):
                robot.stop_motion()
            else:
                robot.send_joint_velocities(q_dot_final, q_current, dt=DT, speed=85)

            # Telemetry
            sys.stdout.write(
                f"\r[STATUS] EE: X:{ee_pos[0]:+.3f} Y:{ee_pos[1]:+.3f} Z:{ee_pos[2]:+.3f} m | "
                f"Gripper: {'[ON]' if gripper.is_active else '[OFF]'}"
            )
            sys.stdout.flush()

            # Διατήρηση loop rate
            elapsed = time.time() - t_start
            to_sleep = DT - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("\n\n[*] Διακοπή από τον χειριστή.")
    finally:
        gripper.cleanup()
        robot.stop_motion()
        print("[+] Το ρομπότ ακινητοποιήθηκε με ασφάλεια.")


if __name__ == "__main__":
    run_teleoperation()
