import time
import numpy as np
from pymycobot import MyArm # type:ignore

from controller_teleop import PS2Controller
from Kinematics_new import KinematicsEngine7DOF

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

Q_MIN = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
Q_MAX = np.array([ 2.79,  1.57,  2.79,  1.57,  2.79,  1.57,  2.79])

# ΣΤΑΣΗ ΕΚΚΙΝΗΣΗΣ ΕΚΤΟΣ ΙΔΙΟΜΟΡΦΙΑΣ (Σχεδόν όρθιο, με ελαφριά κλίση)
Q_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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
        print("\n[AUTO-RESET] Μετάβαση σε Ασφαλή Αρχική Στάση (Εκτός Ιδιομορφίας)...")
        try:
            arm.power_on()
            time.sleep(0.5)
            # Στέλνουμε το ρομπότ στη στάση Q_HOME
            arm.send_radians(Q_HOME.tolist(), 30)
            time.sleep(2.5)
            
            angles = arm.get_radians()
            if angles and len(angles) == DOF:
                q_current = np.array(angles)
            print("[AUTO-RESET COMPLETE] Το ρομπότ είναι έτοιμο και σταθερό!\n")
        except Exception as e:
            print(f"[RESET ERROR] {e}")

    prev_vacuum_state = False

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
                    arm.send_radians(Q_HOME.tolist(), 30)  # Αργή επαναφορά (speed 30)
                    time.sleep(2.5)  # Αναμονή ολοκλήρωσης κίνησης
                    
                    angles = arm.get_radians()
                    if angles and len(angles) == DOF:
                        q_current = np.array(angles)
                    else:
                        q_current = np.copy(Q_HOME)
                else:
                    q_current = np.copy(Q_HOME)
                
                print("[RESET COMPLETE] Το ρομπότ επανήλθε στη μηδενική θέση.\n")
                continue  # Παράκαμψη του υπολοίπου loop για αυτό το frame

            v_cmd = controller.get_cartesian_velocity()
            vacuum_state = controller.process_gripper_events()

            if arm is not None and vacuum_state != prev_vacuum_state:
                arm.set_vacuum_zone(1 if vacuum_state else 0)
                prev_vacuum_state = vacuum_state

            p_end, J = kinematics.compute_fk_and_jacobian(q_current)

            # ΕΚΤΕΛΕΣΗ ΚΙΝΗΣΗΣ ΜΟΝΟ ΑΝ ΥΠΑΡΧΕΙ ΕΝΤΟΛΗ ΑΠΟ ΤΟ ΧΕΙΡΙΣΤΗΡΙΟ
            if np.linalg.norm(v_cmd) > 0:
                q_dot = kinematics.solve_dls_ik(J, v_cmd, q_current)
                q_next = q_current + q_dot * DT
                q_current = np.clip(q_next, Q_MIN, Q_MAX)

                if arm is not None:
                    arm.send_radians(q_current.tolist(), 70)

            v_str = f"Vx:{v_cmd[0]:.2f} Vy:{v_cmd[1]:.2f} Vz:{v_cmd[2]:.2f} Wx: {v_cmd[3]:.2f} Wy:{v_cmd[4]:.2f} Wz:{v_cmd[5]:.2f}"
            p_str = f"XYZ:[{p_end[0]:.2f},{p_end[1]:.2f},{p_end[2]:.2f}]m"
            print(f"{p_str} | {v_str}  ", end="\r")

            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Τερματισμός τηλεχειρισμού.")
        GPIO.output(20, GPIO.HIGH) # Απενεργοποίηση αντλίας
        GPIO.cleanup()

if __name__ == "__main__":
    main()
