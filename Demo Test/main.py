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
    print(f"[HARDWARE SUCCESS] Σύνδεση με MyArm 300 Pi ({SERIAL_PORT})")
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

# Διευρυμένα όρια ασφαλείας για περισσότερη ελευθερία κίνησης
WORKSPACE_LIMITS = {
    'x': (-0.40, 0.40),
    'y': (0.05, 0.45),
    'z': (0.02, 0.50)   # 2 cm πάνω από το τραπέζι
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
        controller = PS2Controller(deadzone=0.20)
        kinematics = KinematicsEngine7DOF()
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης: {e}")
        return

    # --- ΑΥΤΟΜΑΤΟ RESET ΣΤΗΝ ΑΡΧΙΚΗ ΣΤΑΣΗ ΚΑΤΑ ΤΗΝ ΕΚΚΙΝΗΣΗ ---
    q_current = np.zeros(DOF)
    if arm is not None:
        print("\n[AUTO-RESET] Ενεργοποίηση Servos & Επαναφορά στην κατακόρυφη αρχική στάση...")
        try:
            arm.power_on()
            time.sleep(0.5)
            # Στέλνουμε το ρομπότ στην όρθια στάση [0, 0, 0, 0, 0, 0, 0]
            arm.send_radians([0.0]*DOF, 30)
            time.sleep(2.5)  # Αναμονή να φτάσει όρθιο
            
            # Ανάγνωση πραγματικών γωνιών μετά το reset
            angles = arm.get_radians()
            if angles and len(angles) == DOF:
                q_current = np.array(angles)
            print("[AUTO-RESET COMPLETE] Το ρομπότ είναι έτοιμο και όρθιο!\n")
        except Exception as e:
            print(f"[RESET WARNING] Σφάλμα κατά το reset: {e}")

    prev_vacuum_state = False

    print("--- ΤΗΛΕΧΕΙΡΙΣΜΟΣ ΕΝΕΡΓΟΣ (30 Hz) ---")
    print(" - Αριστερός Μοχλός : Κίνηση X (Μπρος/Πίσω) & Y (Αριστερά/Δεξιά)")
    print(" - Δεξιός Μοχλός    : Κίνηση Z (Πάνω/Κάτω) & Περιστροφή Wz")
    print(" - Κουμπί A / B     : Grasp / Release")
    print(" - Πατήστε Ctrl+C για έξοδο.\n")

    try:
        while True:
            loop_start_time = time.time()

            # 1. Read Inputs
            v_cmd = controller.get_cartesian_velocity()
            vacuum_state = controller.process_gripper_events()

            # 2. Hardware Vacuum Control
            if arm is not None and vacuum_state != prev_vacuum_state:
                arm.set_vacuum_zone(1 if vacuum_state else 0)
                prev_vacuum_state = vacuum_state

            # 3. Kinematics & Safety Box
            p_end, J = kinematics.compute_fk_and_jacobian(q_current)
            v_safe = enforce_cartesian_limits(p_end, v_cmd)

            # 4. Motion Dispatch (Αυξήσαμε την ταχύτητα στο 60% για πιο άμεση απόκριση)
            if np.linalg.norm(v_safe) > 0:
                q_dot = kinematics.solve_dls_ik(J, v_safe, q_current)
                q_next = q_current + q_dot * DT
                q_current = np.clip(q_next, Q_MIN, Q_MAX)

                if arm is not None:
                    arm.send_radians(q_current.tolist(), 60) # Ταχύτητα 60%

            # 5. Console Display
            q_deg = np.degrees(q_current)
            q_str = ", ".join([f"{deg:5.1f}°" for deg in q_deg])
            gripper_label = "GRASP [ON]" if vacuum_state else "RELEASE [OFF]"
            print(f"EE Pose: [{p_end[0]:.2f}, {p_end[1]:.2f}, {p_end[2]:.2f}]m | Joints: [{q_str}] | Vacuum: {gripper_label}  ", end="\r")

            # 6. Frequency Sync
            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\n[SHUTDOWN] Τερματισμός τηλεχειρισμού.")
        if arm is not None:
            arm.set_vacuum_zone(0)

if __name__ == "__main__":
    main()
