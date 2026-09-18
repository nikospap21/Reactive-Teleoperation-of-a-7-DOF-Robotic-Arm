import time
import numpy as np
from pymycobot import MyArm #type:ignore
# Εισαγωγή των δύο modules που φτιάξαμε
from controller_teleop import PS2Controller
from kinematics import KinematicsEngine7DOF

SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    print(f"[HARDWARE] Επιτυχής σύνδεση με MyArm 300 Pi στη θύρα {SERIAL_PORT}")
except Exception as e:
    print(f"[HARDWARE ERROR] Αποτυχία σύνδεσης στη θύρα {SERIAL_PORT}: {e}")
    arm = None

# ==========================================
# 1. ΠΑΡΑΜΕΤΡΟΙ ΡΟΜΠΟΤ & ΑΣΦΑΛΕΙΑΣ
# ==========================================
DOF = 7
DT = 1.0 / 30.0  # 30 Hz Control Loop Rate

# Όρια θέσης αρθρώσεων (σε rad)[cite: 1]
Q_MIN = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
Q_MAX = np.array([ 2.79,  1.57,  2.79,  1.57,  2.79,  1.57,  2.79])

# Καρτεσιανό κουτί ασφαλείας (Workspace Limits σε μέτρα)[cite: 1]
WORKSPACE_LIMITS = {
    'x': (-0.35, 0.35),
    'y': (0.10, 0.40),
    'z': (0.02, 0.45)   # z_min = 2cm πάνω από το τραπέζι για ασφάλεια[cite: 1]
}

def enforce_cartesian_limits(p_end, v_cmd):
    """
    Cartesian Workspace Enforcement: 
    Αν το End-Effector αγγίξει όριο workspace, ακυρώνει την ταχύτητα 
    προς την επικίνδυνη κατεύθυνση χωρίς να παγώνει τον κύκλο ελέγχου.
    """
    v_safe = np.copy(v_cmd)

    # Έλεγχος X limits
    if p_end[0] <= WORKSPACE_LIMITS['x'][0] and v_safe[0] < 0:
        v_safe[0] = 0.0
    elif p_end[0] >= WORKSPACE_LIMITS['x'][1] and v_safe[0] > 0:
        v_safe[0] = 0.0

    # Έλεγχος Y limits
    if p_end[1] <= WORKSPACE_LIMITS['y'][0] and v_safe[1] < 0:
        v_safe[1] = 0.0
    elif p_end[1] >= WORKSPACE_LIMITS['y'][1] and v_safe[1] > 0:
        v_safe[1] = 0.0

    # Έλεγχος Z limits (Προστασία Πρόσκρουσης στο Τραπέζι)
    if p_end[2] <= WORKSPACE_LIMITS['z'][0] and v_safe[2] < 0:
        v_safe[2] = 0.0
        print("\n[SAFETY WARNING] Πλησίασμα στο τραπέζι! Η κίνηση προς τα κάτω μπλοκαρίστηκε.", end="")
    elif p_end[2] >= WORKSPACE_LIMITS['z'][1] and v_safe[2] > 0:
        v_safe[2] = 0.0

    return v_safe

# ==========================================
# 2. ΚΥΡΙΟΣ ΚΥΚΛΟΣ ΕΛΕΓΧΟΥ (MAIN TELEOP LOOP)
# ==========================================
def main():
    print("="*60)
    print(" REAL-TIME TELEOPERATION FRAMEWORK - MYARM 300 PI (7-DOF)")
    print("="*60)

    # Αρχικοποίηση Controller & Kinematics Engine
    try:
        controller = PS2Controller()
        kinematics = KinematicsEngine7DOF()
    except Exception as e:
        print(f"Σφάλμα Αρχικοποίησης: {e}")
        return

    # Αρχική κατάσταση αρθρώσεων
    q_current = np.zeros(DOF)

    print("\n[SYSTEM READY] Ο κύκλος ελέγχου εκτελείται στα 30 Hz.")
    print(" - Αριστερός Μοχλός : Κίνηση X / Y")
    print(" - Δεξιός Μοχλός    : Κίνηση Z (Up/Down) & Περιστροφή Yaw")
    print(" - Κουμπί A         : Grasp (Suction ON)")
    print(" - Κουμπί B         : Release (Suction OFF)")
    print(" - Πατήστε Ctrl+C για έξοδο.\n")

    try:
        while True:
            loop_start_time = time.time()

            # 1. Read PS2 Joystick & Gripper Inputs
            v_cmd = controller.get_cartesian_velocity()
            vacuum_state = controller.process_gripper_events()

            # 2. Forward Kinematics & Geometric Jacobian Computation (6x7)
            p_end, J = kinematics.compute_fk_and_jacobian(q_current)

            # 3. Apply Cartesian Safety Box Limits
            v_safe = enforce_cartesian_limits(p_end, v_cmd)

            # 4. Compute Differential IK (DLS + Null-Space Projection)
            if np.linalg.norm(v_safe) > 0:
                q_dot = kinematics.solve_dls_ik(J, v_safe, q_current)

                # Integration για τη νέα θέση αρθρώσεων
                q_next = q_current + q_dot * DT

                # 5. Apply Joint Limits (Clip)
                q_current = np.clip(q_next, Q_MIN, Q_MAX)

            # 6. Real-Time Visual Feedback στο Console
            gripper_label = "GRASPING [ON]" if vacuum_state else "RELEASED [OFF]"
            q_deg = np.degrees(q_current)
            # Αντί για Q1 μόνο, τυπώνουμε όλες τις γωνίες σε μοίρες
            q_str = ", ".join([f"{deg:5.1f}°" for deg in q_deg])
            print(f"EE Pose: [{p_end[0]:.2f}, {p_end[1]:.2f}, {p_end[2]:.2f}]m | Joints: [{q_str}] | Gripper: {gripper_label}  ", end="\r")

            # 7. Maintain 30 Hz Cycle Frequency
            elapsed = time.time() - loop_start_time
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("[SYSTEM SHUTDOWN] Ο κύκλος ελέγχου τερματίστηκε με ασφάλεια.")
        print("="*60)

if __name__ == "__main__":
    main()