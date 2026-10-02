import time
import numpy as np
from pymycobot import MyArm
import RPi.GPIO as GPIO

from controller_teleop import PS2Controller
from kinematics import KinematicsEngine7DOF

# 1. Σύνδεση Raspberry Pi -> MyArm
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)
    print(f"SUCCESS: Σύνδεση με MyArm 300 Pi ({SERIAL_PORT})")
except Exception as e:
    print(f"ERROR: Αποτυχία σύνδεσης: {e}")
    arm = None

# 2. ΠΑΡΑΜΕΤΡΟΙ ΡΟΜΠΟΤ & ΑΣΦΑΛΕΙΑΣ
DOF = 7
DT = 1.0 / 30.0

Q_MIN = np.array([-2.7925, -1.2217, -2.9670, -1.9722, -2.9670, -2.0071, -3.14159])
Q_MAX = np.array([ 2.7925,  2.0071,  2.9670,  1.3089,  2.9670,  2.0071,  3.14159])

EE_Offset = 0.15
WS_X_MIN, WS_X_MAX = -0.38 - EE_Offset, 0.38 + EE_Offset
WS_Y_MIN, WS_Y_MAX = -0.38 - EE_Offset, 0.38 + EE_Offset
WS_Z_MIN, WS_Z_MAX =  -0.08, 0.38 + EE_Offset 
# Όλα είναι μετρημένα στο εργαστήριο και το EE_Offset και το Base-Table

Q_HOME = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0])
#Q_HOME = np.array([1.5, 0.5, 0.0, 0.0, 0.0, -1.5, 0.0])  #R
#Q_HOME = np.array([-1.5, 0.5, -0.5, -0.5, 0.0, -1.5, 0.0])  #L

# 3. Main
def main():
    try:
        controller = PS2Controller(deadzone=0.0)
        kinematics = KinematicsEngine7DOF()
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης: {e}")
        return

    # Reset
    q_current = np.copy(Q_HOME)
    if arm is not None:
        print("\nRESET: Μετάβαση σε Ασφαλή Αρχική Στάση...")
        try:
            arm.power_on()
            time.sleep(0.5)
            arm.send_radians(Q_HOME.tolist(), 30)
            time.sleep(2.5)
            
            angles = arm.get_radians()
            if angles and len(angles) == DOF:
                q_current = np.array(angles)
            print("RESETCOMPLETE: Το ρομπότ είναι έτοιμο και σταθερό!\n")
        except Exception as e:
            print(f"RESETERROR: {e}")

    print("--- ΤΗΛΕΧΕΙΡΙΣΜΟΣ ΕΝΕΡΓΟΣ (30 Hz) ---")
    print("Αφήστε τους μοχλούς: Το ρομπότ ΠΡΕΠΕΙ να μένει ΑΠΟΛΥΤΑ ΑΚΙΝΗΤΟ.\n")

    try:
        while True:
            loop_start_time = time.time()

            if controller.process_reset_event():
                if arm is not None:
                    arm.power_on()
                    time.sleep(0.2)
                    arm.send_radians(Q_HOME.tolist(), 30)
                    time.sleep(2.5)
                    
                    angles = arm.get_radians()
                    if angles and len(angles) == DOF:
                        q_current = np.array(angles)
                    else:
                        q_current = np.copy(Q_HOME)
                else:
                    q_current = np.copy(Q_HOME)
                
                print("RESETCOMPLETE: Το ρομπότ επανήλθε στη μηδενική θέση.\n")
                continue

            # Υπολογισμός FK & Jacobian
            p_end, R_end , J = kinematics.compute_fk_and_jacobian(q_current)

            # Μετατροπές Ταχύτητας
            v_raw = controller.get_cartesian_velocity()

            v_base_lin = R_end @ v_raw[0:3]
            v_base_ang = R_end @ v_raw[3:6]
            v_cmd = np.concatenate([v_base_lin, v_base_ang])

            # Υπολογισμός vaccum_gripper
            vaccum_state = controller.process_gripper_events()

            # Επίλυση IK ΜΟΝΟ υπό τηλεχειρισμό
            if np.linalg.norm(v_cmd) > 0:
                try:
                    q_dot = kinematics.solve_dls_ik(J, v_cmd, q_current)
                    if np.any(np.isnan(q_dot)) or np.any(np.isinf(q_dot)):
                        q_dot = np.zeros(DOF)
                except Exception:
                    q_dot = np.zeros(DOF)

                is_moving_xyz = np.linalg.norm(v_cmd[0:3]) > 0.02
                
                if abs(v_cmd[5]) > 0.04 and not is_moving_xyz:
                    q_dot[0:6] = 0.0
                    q_dot[6] = v_cmd[5]

                q_next = q_current + q_dot * DT
                
                q_next = np.clip(q_next, Q_MIN, Q_MAX)

                p_next, R_end, J = kinematics.compute_fk_and_jacobian(q_next)
                in_workspace = (
                    WS_X_MIN <= p_next[0] <= WS_X_MAX and
                    WS_Y_MIN <= p_next[1] <= WS_Y_MAX and
                    WS_Z_MIN <= p_next[2] <= WS_Z_MAX
                )

                if in_workspace:
                    q_current = q_next
                    if arm is not None:
                        arm.send_radians(q_current.tolist(), 30)
                else:
                    print("\nWORKSPACE LIMIT! Motion Blocked.  ", end="\r")

            # εμφάνιση κατάστασης
            v_str = f"Vx:{v_cmd[0]:.2f} Vy:{v_cmd[1]:.2f} Vz:{v_cmd[2]:.2f} Wx:{v_cmd[3]:.2f} Wy:{v_cmd[4]:.2f} Wz:{v_cmd[5]:.2f}"
            p_str = f"XYZ:[{p_end[0]:.2f},{p_end[1]:.2f},{p_end[2]:.2f}]m"
            Q_str = f"Q (rad): [{q_current[0]:.2f}, {q_current[1]:.2f}, {q_current[2]:.2f},{q_current[3]:.2f}, {q_current[4]:.2f}, {q_current[5]:.2f}, {q_current[6]:.2f}]"

            print(f"{p_str} | {Q_str}", end ="\r")

            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\nSHUTDOWN: Τερματισμός τηλεχειρισμού.")
        GPIO.output(20, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
