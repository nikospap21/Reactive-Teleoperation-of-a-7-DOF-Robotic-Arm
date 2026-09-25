import time
import numpy as np
from pymycobot import MyArm # type:ignore
import RPi.GPIO as GPIO # type:ignore

from controller_teleop import PS2Controller
from kinematics_new import KinematicsEngine7DOF

# ==========================================
# 1. ΕΣΩΤΕΡΙΚΗ ΣΥΝΔΕΣΗ (Raspberry Pi -> MyArm)
# ==========================================
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)
    print(f"[HARDWARE SUCCESS] Σύνδεση με MyArm 300 Pi ({SERIAL_PORT})")
except Exception as e:
    print(f"[HARDWARE ERROR] Αποτυχία σύνδεσης: {e}")
    arm = None

# ==========================================
# 2. ΠΑΡΑΜΕΤΡΟΙ ΡΟΜΠΟΤ & ΑΣΦΑΛΕΙΑΣ
# ==========================================
DOF = 7
DT = 1.0 / 30.0

# Α) Γωνιακά Όρια Αρθρώσεων (σε rad)
Q_MIN = np.array([-2.7925, -1.2217, -2.9670, -1.9722, -2.9670, -2.0071, -3.14159])
Q_MAX = np.array([ 2.7925,  2.0071,  2.9670,  1.3089,  2.9670,  2.0071,  3.14159])

# Β) Όρια Γωνιακής Ταχύτητας Αρθρώσεων (σε rad/s)
Q_DOT_MAX = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5])

# Γ) Όρια Χώρου Εργασίας / Workspace Limits (σε μέτρα)
WS_X_MIN, WS_X_MAX = -0.35, 0.35
WS_Y_MIN, WS_Y_MAX = -0.35, 0.35
WS_Z_MIN, WS_Z_MAX =  0.02, 0.45  # Z_min = 0.02m (προστασία από πρόσκρουση στο τραπέζι)

# Στάση Εκκίνησης (Home)
Q_HOME = np.array([0.0, 0.5, 0.0, -0.5, 0.0, 0.3, 0.0])
#Q_HOME = np.array([1.5, 0.5, 0.0, 0.0, 0.0, -1.5, 0.0])  #Right
#Q_HOME = np.array([-1.5, 0.5, -0.5, -0.5, 0.0, -1.5, 0.0])  #Left


# ==========================================
# 3. ΚΥΡΙΟΣ ΚΥΚΛΟΣ ΕΛΕΓΧΟΥ (MAIN LOOP)
# ==========================================
def main():
    try:
        controller = PS2Controller(deadzone=0.1)
        kinematics = KinematicsEngine7DOF()
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης: {e}")
        return

    # AUTO-RESET ΣΤΗΝ ΑΣΦΑΛΗ ΑΡΧΙΚΗ ΘΕΣΗ
    q_current = np.copy(Q_HOME)
    if arm is not None:
        print("\n[AUTO-RESET] Μετάβαση σε Ασφαλή Αρχική Στάση...")
        try:
            arm.power_on()
            time.sleep(0.5)
            arm.send_radians(Q_HOME.tolist(), 30)
            time.sleep(2.5)
            
            angles = arm.get_radians()
            if angles and len(angles) == DOF:
                q_current = np.array(angles)
            print("[AUTO-RESET COMPLETE] Το ρομπότ είναι έτοιμο και σταθερό!\n")
        except Exception as e:
            print(f"[RESET ERROR] {e}")

    print("--- ΤΗΛΕΧΕΙΡΙΣΜΟΣ ΕΝΕΡΓΟΣ (30 Hz) ---")
    print(" - Αφήστε τους μοχλούς: Το ρομπότ ΠΡΕΠΕΙ να μένει ΑΠΟΛΥΤΑ ΑΚΙΝΗΤΟ.\n")

    try:
        while True:
            loop_start_time = time.time()

            # 1. Έλεγχος για RESET EVENT (Κουμπί Start/X)
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
                
                print("[RESET COMPLETE] Το ρομπότ επανήλθε στη μηδενική θέση.\n")
                continue

            # 2. Input Processing & Vacuum Control
            # Υπολογισμός FK, IK, Rotation Matrix και Jacobian
            p_end, J = kinematics.compute_fk_and_jacobian(q_current)

            # Μετατροπές Ταχύτητας (Tool Frame -> Base Frame)
            v_cmd = controller.get_cartesian_velocity()
            #v_cmd_base = kinematics.transform_v_to_base(v_cmd_tool, R_end)

            # ΕΚΤΕΛΕΣΗ ΚΙΝΗΣΗΣ ΜΟΝΟ ΑΝ ΥΠΑΡΧΕΙ ΕΝΤΟΛΗ ΑΠΟ ΤΟ ΧΕΙΡΙΣΤΗΡΙΟ
            if np.linalg.norm(v_cmd) > 0:
                try:
                    # α) Επίλυση DLS IK
                    q_dot = kinematics.solve_dls_ik(J, v_cmd, q_current)
                    if np.any(np.isnan(q_dot)) or np.any(np.isinf(q_dot)):
                        q_dot = np.zeros(DOF)
                except Exception:
                    q_dot = np.zeros(DOF)

                # β) Joint Velocity Clamping (Περιορισμός γωνιακής ταχύτητας)
                q_dot = np.clip(q_dot, -Q_DOT_MAX, Q_DOT_MAX)

                # γ) Υπολογισμός νέας στάσης
                q_next = q_current + q_dot * DT
                
                # δ) Joint Limits Clamping (Περιορισμός γωνιών αρθρώσεων)
                q_next = np.clip(q_next, Q_MIN, Q_MAX)

                # ε) Έλεγχος Workspace Limits μέσω Forward Kinematics
                p_next, _ = kinematics.compute_fk_and_jacobian(q_next)
                in_workspace = (
                    WS_X_MIN <= p_next[0] <= WS_X_MAX and
                    WS_Y_MIN <= p_next[1] <= WS_Y_MAX and
                    WS_Z_MIN <= p_next[2] <= WS_Z_MAX
                )

                if in_workspace:
                    q_current = q_next
                    if arm is not None:
                        arm.send_radians(q_current.tolist(), 70)
                else:
                    print("\n[WARNING] Reach Workspace Limit! Motion Blocked.  ", end="\r")

            v_str = f"Vx:{v_cmd[0]:.2f} Vy:{v_cmd[1]:.2f} Vz:{v_cmd[2]:.2f} Wx:{v_cmd[3]:.2f} Wy:{v_cmd[4]:.2f} Wz:{v_cmd[5]:.2f}"
            p_str = f"XYZ:[{p_end[0]:.2f},{p_end[1]:.2f},{p_end[2]:.2f}]m"
            print(f"{p_str} | {v_str}  ", end="\r")

            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Τερματισμός τηλεχειρισμού.")
        # Απενεργοποίηση αντλίας (20=LOW) και καθαρισμός GPIO
        GPIO.output(20, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
