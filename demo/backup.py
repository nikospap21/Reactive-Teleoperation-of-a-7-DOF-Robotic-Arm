import time
import numpy as np
from pymycobot import MyArm # type:ignore

from controller_teleop import PS2Controller
from kinematics import KinematicsEngine7DOF

# ==========================================
# 1. ΕΣΩΤΕΡΙΚΗ ΣΥΝΔΕΣΗ (Raspberry Pi -> MyArm)
# ==========================================
SERIAL_PORT = "/dev/ttyAMA0"  # Εσωτερική θύρα Raspberry Pi
BAUDRATE = 115200             # Baudrate επικοινωνίας

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)  # Αναμονή σταθεροποίησης σειριακής
    
    # ΕΝΕΡΓΟΠΟΙΗΣΗ ΚΙΝΗΤΗΡΩΝ (Ξεκλείδωμα αρθρώσεων)
    arm.power_on()
    time.sleep(0.5)
    print(f"[HARDWARE SUCCESS] Σύνδεση & Ενεργοποίηση Servos στο MyArm 300 Pi ({SERIAL_PORT})")
except Exception as e:
    print(f"[HARDWARE ERROR] Αποτυχία σύνδεσης: {e}")
    arm = None

# ==========================================
# 2. ΠΑΡΑΜΕΤΡΟΙ ΡΟΜΠΟΤ & ΑΣΦΑΛΕΙΑΣ
# ==========================================
DOF = 7
DT = 1.0 / 30.0  # 30 Hz Control Loop Rate

# Όρια θέσης αρθρώσεων (rad)
Q_MIN = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
Q_MAX = np.array([ 2.79,  1.57,  2.79,  1.57,  2.79,  1.57,  2.79])

# Safety Box (Workspace Limits σε μέτρα)
WORKSPACE_LIMITS = {
    'x': (-0.35, 0.35),
    'y': (0.10, 0.40),
    'z': (0.02, 0.45)   # 2 cm πάνω από το τραπέζι
}

import time
import numpy as np
from pymycobot import MyArm # type:ignore

from controller_teleop import PS2Controller
from kinematics import KinematicsEngine7DOF

# ==========================================
# 1. ΕΣΩΤΕΡΙΚΗ ΣΥΝΔΕΣΗ (Raspberry Pi -> MyArm)
# ==========================================
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)
    
    # ΕΝΕΡΓΟΠΟΙΗΣΗ ΚΙΝΗΤΗΡΩΝ
    arm.power_on()
    time.sleep(0.5)
    print(f"[HARDWARE SUCCESS] Σύνδεση & Ενεργοποίηση Servos στο MyArm 300 Pi ({SERIAL_PORT})")
except Exception as e:
    print(f"[HARDWARE ERROR] Αποτυχία σύνδεσης: {e}")
    arm = None

# ==========================================
# 2. ΠΑΡΑΜΕΤΡΟΙ ΡΟΜΠΟΤ & ΑΣΦΑΛΕΙΑΣ
# ==========================================
DOF = 7
DT = 1.0 / 30.0  # 30 Hz Rate

Q_MIN = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
Q_MAX = np.array([ 2.79,  1.57,  2.79,  1.57,  2.79,  1.57,  2.79])

WORKSPACE_LIMITS = {
    'x': (-0.35, 0.35),
    'y': (0.10, 0.40),
    'z': (0.02, 0.45)   # 2 cm πάνω από το τραπέζι
}

def enforce_cartesian_limits(p_end, v_cmd):
    v_safe = np.copy(v_cmd)
    if p_end[0] <= WORKSPACE_LIMITS['x'][0] and v_safe[0] < 0: v_safe[0] = 0.0
    elif p_end[0] >= WORKSPACE_LIMITS['x'][1] and v_safe[0] > 0: v_safe[0] = 0.0

    if p_end[1] <= WORKSPACE_LIMITS['y'][0] and v_safe[1] < 0: v_safe[1] = 0.0
    elif p_end[1] >= WORKSPACE_LIMITS['y'][1] and v_safe[1] > 0: v_safe[1] = 0.0

    if p_end[2] <= WORKSPACE_LIMITS['z'][0] and v_safe[2] < 0: v_safe[2] = 0.0
    elif p_end[2] >= WORKSPACE_LIMITS['z'][1] and v_safe[2] > 0: v_safe[2] = 0.0

    return v_safe

# ==========================================
# 3. ΚΥΡΙΟΣ ΚΥΚΛΟΣ ΕΛΕΓΧΟΥ (MAIN LOOP)
# ==========================================
def main():
    try:
        # Αυξάνουμε το deadzone στο 0.20 για απόλυτη ασφάλεια από θόρυβο
        controller = PS2Controller(deadzone=0.20)
        kinematics = KinematicsEngine7DOF()
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης: {e}")
        return

    # ΑΥΣΤΗΡΗ ΑΝΑΓΝΩΣΗ ΑΡΧΙΚΗΣ ΘΕΣΗΣ ΡΟΜΠΟΤ (SYNC)
    q_current = None
    if arm is not None:
        print("[HARDWARE] Ανάγνωση αρχικής θέσης βραχίονα...")
        for attempt in range(5):
            try:
                angles = arm.get_radians()
                if angles and len(angles) == DOF:
                    q_current = np.array(angles)
                    print(f"[HARDWARE SUCCESS] Αρχικές γωνίες: {np.degrees(q_current).round(1)}°")
                    break
            except Exception:
                pass
            time.sleep(0.2)

    if q_current is None:
        print("[WARNING] Αποτυχία ανάγνωσης γωνιών! Χρήση default στάσης ασφαλείας.")
        q_current = np.array([0.0, 0.2, 0.0, 0.5, 0.0, 0.2, 0.0]) # Όχι όλα 0!

    prev_vacuum_state = False

    print("\n--- ΤΗΛΕΧΕΙΡΙΣΜΟΣ ΕΝΕΡΓΟΣ ---")
    print(" - Αφήστε τους μοχλούς: Το ρομπότ ΠΡΕΠΕΙ να μένει ΑΚΙΝΗΤΟ.")
    print(" - Πατήστε Ctrl+C για άμεσο τερματισμό.\n")

    try:
        while True:
            loop_start_time = time.time()

            # 1. Read Inputs
            v_cmd = controller.get_cartesian_velocity()
            vacuum_state = controller.process_gripper_events()

            # 2. ΑΥΣΤΗΡΟ ΦΙΛΤΡΑΡΙΣΜΑ DRIFT (Κατώφλι ακινησίας)
            # Αν όλες οι ταχύτητες είναι πολύ μικρές, τις ΜΗΔΕΝΙΖΟΥΜΕ
            if np.max(np.abs(v_cmd)) < 0.01:
                v_cmd = np.zeros(6)

            # 3. Hardware Vacuum Control
            if arm is not None and vacuum_state != prev_vacuum_state:
                arm.set_vacuum_zone(1 if vacuum_state else 0)
                prev_vacuum_state = vacuum_state

            # 4. Kinematics & Safety Box
            p_end, J = kinematics.compute_fk_and_jacobian(q_current)
            v_safe = enforce_cartesian_limits(p_end, v_cmd)

            # 5. ΜΟΝΟ ΑΝ ΥΠΑΡΧΕΙ ΠΡΑΓΜΑΤΙΚΗ ΕΝΤΟΛΗ ΚΙΝΕΙΤΑΙ ΤΟ ΡΟΜΠΟΤ
            if np.linalg.norm(v_safe) > 0:
                q_dot = kinematics.solve_dls_ik(J, v_safe, q_current)
                q_next = q_current + q_dot * DT
                q_current = np.clip(q_next, Q_MIN, Q_MAX)

                # Αποστολή στο ρομπότ με χαμηλότερη, ασφαλή ταχύτητα (speed 30%)
                if arm is not None:
                    arm.send_radians(q_current.tolist(), 30)

            # 6. Console Display
            q_deg = np.degrees(q_current)
            q_str = ", ".join([f"{deg:5.1f}°" for deg in q_deg])
            gripper_label = "GRASP [ON]" if vacuum_state else "RELEASE [OFF]"
            print(f"EE Pose: [{p_end[0]:.2f}, {p_end[1]:.2f}, {p_end[2]:.2f}]m | Joints: [{q_str}] | Vacuum: {gripper_label}  ", end="\r")

            # 7. Sync
            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Τερματισμός τηλεχειρισμού.")
        if arm is not None:
            arm.set_vacuum_zone(0)

if __name__ == "__main__":
    main()

