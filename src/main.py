import time
import numpy as np
from pymycobot import MyArm # type:ignore
import RPi.GPIO as GPIO # type:ignore

from controller_teleop import PS2Controller
from test import KinematicsEngine7DOF

SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)
    print(f"[HARDWARE SUCCESS] Σύνδεση με MyArm 300 Pi ({SERIAL_PORT})")
except Exception as e:
    print(f"[HARDWARE ERROR] Αποτυχία σύνδεσης: {e}")
    arm = None

DOF = 7
DT = 1.0 / 30.0

EE_OFFSET = 0.15
kinematics = KinematicsEngine7DOF(ee_offset=EE_OFFSET)

Q_MIN = kinematics.q_min
Q_MAX = kinematics.q_max

# Όρια Χώρου Εργασίας (ενσωματώνουν το ύψος της βεντούζας)
WS_X_MIN, WS_X_MAX = -0.35, 0.35
WS_Y_MIN, WS_Y_MAX = -0.35, 0.35
WS_Z_MIN, WS_Z_MAX =  0.02, 0.45   # 2cm ασφαλές όριο πάνω από την επιφάνεια του τραπεζιού

# Ασφαλής Στάση Έναρξης (βεντούζα στραμμένη προς τα κάτω)
Q_HOME = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0])

def main():
    try:
        controller = PS2Controller(deadzone=0.08)
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης Χειριστηρίου: {e}")
        return

    q_current = np.copy(Q_HOME)
    if arm is not None:
        print("\n[AUTO-RESET] Μετάβαση σε Αρχική Στάση...")
        try:
            arm.power_on()
            time.sleep(0.5)
            arm.send_radians(Q_HOME.tolist(), 30)
            time.sleep(2.5)
            angles = arm.get_radians()
            if angles and len(angles) == DOF:
                q_current = np.array(angles)
            print("[AUTO-RESET COMPLETE] Ο βραχίονας είναι έτοιμος!\n")
        except Exception as e:
            print(f"[RESET ERROR] {e}")

    print("--- ΤΗΛΕΧΕΙΡΙΣΜΟΣ ΕΝΕΡΓΟΣ (30 Hz) ---")

    try:
        while True:
            t_start = time.time()

            # 1. Reset Event
            if controller.process_reset_event():
                if arm is not None:
                    arm.power_on()
                    arm.send_radians(Q_HOME.tolist(), 30)
                    time.sleep(2.0)
                    angles = arm.get_radians()
                    if angles and len(angles) == DOF:
                        q_current = np.array(angles)
                    else:
                        q_current = np.copy(Q_HOME)
                else:
                    q_current = np.copy(Q_HOME)
                continue

            # 2. Ανάγνωση Χειριστηρίου & Βεντούζας
            v_raw = controller.get_cartesian_velocity()
            controller.process_gripper_events()

            # Οι γραμμικές ταχύτητες ορίζονται απευθείας στο Base Frame για σταθερό έλεγχο
            # [vx, vy, vz, wx, wy, wz]
            v_cmd = np.copy(v_raw)

            # 3. Forward Kinematics & Έλεγχος Workspace
            p_end, R_end, J = kinematics.compute_fk_and_jacobian(q_current)

            v_safe = np.copy(v_cmd)
            if (p_end[0] <= WS_X_MIN and v_safe[0] < 0) or (p_end[0] >= WS_X_MAX and v_safe[0] > 0):
                v_safe[0] = 0.0
            if (p_end[1] <= WS_Y_MIN and v_safe[1] < 0) or (p_end[1] >= WS_Y_MAX and v_safe[1] > 0):
                v_safe[1] = 0.0
            if (p_end[2] <= WS_Z_MIN and v_safe[2] < 0) or (p_end[2] >= WS_Z_MAX and v_safe[2] > 0):
                v_safe[2] = 0.0

            # 4. Επίλυση Κινηματικής
            if np.linalg.norm(v_safe) > 0.005:
                # Decoupling: Καθαρό Yaw στο Joint 7 αν δεν υπάρχει μεταφορική κίνηση
                is_translating = np.linalg.norm(v_safe[0:3]) > 0.01
                if abs(v_safe[5]) > 0.02 and not is_translating:
                    q_dot = np.zeros(DOF)
                    q_dot[6] = v_safe[5]
                else:
                    q_dot = kinematics.solve_dls_ik(J, v_safe, q_current, lock_pitch_roll=True)

                if np.any(np.isnan(q_dot)) or np.any(np.isinf(q_dot)):
                    q_dot = np.zeros(DOF)

                # Ενημέρωση και Clamping
                q_next = q_current + q_dot * DT
                q_next = np.clip(q_next, Q_MIN, Q_MAX)

                # Έλεγχος ότι η επόμενη στάση παραμένει εντός ορίων
                p_next, _, _ = kinematics.compute_fk_and_jacobian(q_next)
                in_ws = (WS_X_MIN <= p_next[0] <= WS_X_MAX and
                         WS_Y_MIN <= p_next[1] <= WS_Y_MAX and
                         WS_Z_MIN <= p_next[2] <= WS_Z_MAX)

                if in_ws:
                    q_current = q_next
                    if arm is not None:
                        arm.send_radians(q_current.tolist(), 40)
                else:
                    print("\n[WARNING] Επαφή με όριο χώρου εργασίας! Κίνηση αποκλείστηκε.", end="\r")

            # Τηλεμετρία
            p_str = f"XYZ: [{p_end[0]:+.3f}, {p_end[1]:+.3f}, {p_end[2]:+.3f}]m"
            v_str = f"Vx:{v_safe[0]:+.2f} Vy:{v_safe[1]:+.2f} Vz:{v_safe[2]:+.2f} Wz:{v_safe[5]:+.2f}"
            print(f"{p_str} | {v_str}  ", end="\r")

            elapsed = time.time() - t_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Τερματισμός τηλεχειρισμού...")
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
