import numpy as np

class KinematicsEngine7DOF:
    def __init__(self, ee_offset=0.15):
        """
        Κινηματική Μηχανή βασισμένη στις ακριβείς διαστάσεις του MyArm 300 Pi URDF.
        ee_offset: Το φυσικό μήκος της βεντούζας (0.15 m).
        """
        self.dof = 7
        self.ee_offset = ee_offset

        # Πραγματικά ασύμμετρα όρια αρθρώσεων σε ακτίνια
        self.q_min = np.array([-2.7925, -1.2217, -2.9670, -1.9722, -2.9670, -2.0071, -3.14159])
        self.q_max = np.array([ 2.7925,  2.0071,  2.9670,  1.3089,  2.9670,  2.0071,  3.14159])
        self.q_mid = (self.q_max + self.q_min) / 2.0
        self.q_range = self.q_max - self.q_min

        # Στάση ανάπαυσης (Rest pose για null-space)
        self.q_rest = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0])

        # Παράμετροι απόσβεσης DLS
        self.lambda_max = 0.04
        self.singularity_threshold = 0.02
        self.k_null = 0.5
        self.max_joint_vel = 1.0

        # URDF Joint Origins: translation [x, y, z] και RPY [roll, pitch, yaw]
        self.joint_origins = [
            {"xyz": np.array([0.0, 0.0, 0.1405]),     "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, 0.0, 0.0]),        "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, -0.110, 0.0]),     "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, 0.0, 0.0]),        "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, -0.126, 0.0]),     "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, 0.0, 0.0]),        "rpy": np.array([0.0, 0.0, 0.0])},
            {"xyz": np.array([0.0, 0.0, 0.060]),      "rpy": np.array([0.0, 0.0, 0.0])},
        ]

        # Άξονες περιστροφής στο τοπικό σύστημα κάθε συνδέσμου
        self.joint_axes = [
            np.array([0.0, 0.0, 1.0]),  # Joint 1: Base Yaw
            np.array([0.0, 1.0, 0.0]),  # Joint 2: Shoulder Pitch
            np.array([0.0, 0.0, 1.0]),  # Joint 3: Elbow Roll
            np.array([0.0, 1.0, 0.0]),  # Joint 4: Elbow Pitch
            np.array([0.0, 0.0, 1.0]),  # Joint 5: Wrist Roll
            np.array([0.0, 1.0, 0.0]),  # Joint 6: Wrist Pitch
            np.array([0.0, 0.0, 1.0])   # Joint 7: Tool Yaw / Flange
        ]

    @staticmethod
    def _rpy_to_matrix(rpy):
        r, p, y = rpy
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def _rodrigues(axis, angle):
        """Περιστροφή γύρω από αυθαίρετο μοναδιαίο άξονα."""
        axis = axis / np.linalg.norm(axis)
        kx, ky, kz = axis
        K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
        return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)

    def compute_fk_and_jacobian(self, q):
        """
        Υπολογίζει:
        - p_end: Θέση άκρου βεντούζας (μαζί με το ee_offset)
        - R_end: Προσανατολισμός τελικού δράστη
        - J: Γεωμετρικός Ιακωβιανός 6x7
        """
        T = np.eye(4)
        origins = [np.zeros(3)]
        z_axes = []

        for i in range(self.dof):
            # 1. Σταθερός μετασχηματισμός συνδέσμου
            R_orig = self._rpy_to_matrix(self.joint_origins[i]["rpy"])
            p_orig = self.joint_origins[i]["xyz"]
            T_orig = np.eye(4)
            T_orig[0:3, 0:3] = R_orig
            T_orig[0:3, 3] = p_orig
            T = T @ T_orig

            # 2. Καταγραφή άξονα περιστροφής στο σύστημα βάσης
            axis_world = T[0:3, 0:3] @ self.joint_axes[i]
            axis_world = axis_world / np.linalg.norm(axis_world)
            z_axes.append(axis_world)
            origins.append(np.copy(T[0:3, 3]))

            # 3. Περιστροφή άρθρωσης q[i]
            R_joint = self._rodrigues(self.joint_axes[i], q[i])
            T_joint = np.eye(4)
            T_joint[0:3, 0:3] = R_joint
            T = T @ T_joint

        # Προσθήκη offset βεντούζας κατά τον τοπικό Z άξονα του Joint 7
        p_flange = T[0:3, 3]
        R_end = T[0:3, 0:3]
        p_end = p_flange + R_end @ np.array([0.0, 0.0, self.ee_offset])

        # Κατασκευή Γεωμετρικού Ιακωβιανού (6 x 7)
        J = np.zeros((6, self.dof))
        for i in range(self.dof):
            p_joint = origins[i + 1]
            z_i = z_axes[i]
            J[0:3, i] = np.cross(z_i, p_end - p_joint)  # Γραμμική ταχύτητα
            J[3:6, i] = z_i                           # Γωνιακή ταχύτητα

        return p_end, R_end, J

    def solve_dls_ik(self, J, v_cmd, q_current, lock_pitch_roll=True):
        """
        Επίλυση διαφορικής κινηματικής μέσω Damped Least Squares και Null-Space.
        Αν lock_pitch_roll=True, διατηρεί μηδενικές τις αποκλίσεις κλίσης του καρπού.
        """
        JJt = J @ J.T
        det_JJt = np.linalg.det(JJt)
        w = np.sqrt(max(0.0, det_JJt))

        # Προσαρμοστική απόσβεση Yoshikawa
        if w < self.singularity_threshold:
            damping = self.lambda_max * (1.0 - (w / self.singularity_threshold) ** 2)
        else:
            damping = 0.001

        # Regularized Inversion
        damp_mat = (damping ** 2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJt + damp_mat)

        # Επιβολή Z-down / Pitch-Roll σταθεροποίησης αν δεν ζητείται περιστροφή
        v_task = np.copy(v_cmd)
        if lock_pitch_roll and np.linalg.norm(v_cmd[3:5]) == 0.0:
            v_task[3] = 0.0
            v_task[4] = 0.0

        q_dot_task = J_pinv @ v_task

        # Null-Space Projection (Αποφυγή ορίων & επιστροφή σε q_rest)
        grad_limits = (q_current - self.q_mid) / (self.q_range ** 2)
        q_null_target = -1.2 * grad_limits + self.k_null * (self.q_rest - q_current)

        I7 = np.eye(self.dof)
        null_proj = I7 - (J_pinv @ J)
        q_dot_null = null_proj @ q_null_target

        q_dot = q_dot_task + q_dot_null

        # Ομοιόμορφος κορεσμός (Uniform Scaling) για αποτροπή παραμορφώσεων τροχιάς
        max_v = np.max(np.abs(q_dot))
        if max_v > self.max_joint_vel:
            q_dot = q_dot * (self.max_joint_vel / max_v)

        return q_dot
