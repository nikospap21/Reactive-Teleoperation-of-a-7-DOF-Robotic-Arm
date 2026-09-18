import math
import numpy as np

def pretty(T, name):
    print(f"\n{name}:\n", np.array2string(T, precision=4, suppress_small=True))

class KinematicsEngine7DOF:
    def __init__(self, lambda_sq=0.005, k_null=0.5):
        """
        Αρχικοποίηση Κινηματικής Μηχανής για το MyArm 300 Pi (7-DOF)
        """
        # DH Parameters [a_i-1, alpha_i-1, d_i, theta_offset]
        self.dh_params = [
            [0.0,      0.0,         0.1405,  0.0],  # Joint 1
            [0.0,     -math.pi/2,   0.0,     0.0],  # Joint 2
            [0.0,      math.pi/2,   0.1200,  0.0],  # Joint 3
            [0.0,     -math.pi/2,   0.0,     0.0],  # Joint 4
            [0.0,      math.pi/2,   0.1200,  0.0],  # Joint 5
            [0.0,     -math.pi/2,   0.0,     0.0],  # Joint 6
            [0.0,      math.pi/2,   0.0600,  0.0]   # Joint 7 (End-Effector)
        ]

        self.dof = 7
        self.lambda_sq = lambda_sq  # Damping factor για DLS (Singularity handling)
        self.k_null = k_null        # Gain για το Null Space Optimization
        self.q_rest = np.zeros(self.dof)  # Αρχική / Άνετη στάση (Rest posture)

    def _dh_matrix(self, a, alpha, d, theta):
        """Υπολογισμός 4x4 Homogeneous Transformation Matrix (Standard DH)."""
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0.0,    sa,     ca,    d],
            [0.0,   0.0,    0.0,  1.0]
        ])

    def compute_fk_and_jacobian(self, q):
        """
        Υπολογισμός Forward Kinematics (FK) και Geometric Jacobian (6x7).
        - FK: Επιστρέφει τη θέση p_end [x, y, z] του End-Effector.
        - Jacobian: 6 γραμμές (3 linear, 3 angular) x 7 στήλες.
        """
        T = np.eye(4)
        positions = [np.zeros(3)]
        z_axes = [np.array([0.0, 0.0, 1.0])]

        # Forward Pass
        for i in range(self.dof):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = q[i] + theta_offset
            T_i = self._dh_matrix(a, alpha, d, theta)
            T = T @ T_i
            positions.append(T[0:3, 3])
            z_axes.append(T[0:3, 2])

        p_end = positions[-1]
        J = np.zeros((6, self.dof))

        # Geometric Jacobian Construction (6x7)
        for i in range(self.dof):
            z_i = z_axes[i]
            p_i = positions[i]
            J[0:3, i] = np.cross(z_i, p_end - p_i)  # Linear Velocity Part (3x7)
            J[3:6, i] = z_i                         # Angular Velocity Part (3x7)

        return p_end, J

    def solve_dls_ik(self, J, v_cmd, q_current):
        """
        Differential Inverse Kinematics με Damped Least Squares (DLS) 
        και Redundancy Resolution μέσω Null-Space Projection.
        """
        # 1. Damped Pseudo-Inverse: J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
        JJt = J @ J.T
        damp_identity = self.lambda_sq * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJt + damp_identity)

        # 2. Main Kinematic Task (Κίνηση End-Effector)
        q_dot_task = J_pinv @ v_cmd

        # 3. Null-Space Projection (Αξιοποίηση περιττής άρθρωσης 7-DOF)
        # N = I - J_pinv * J
        null_projector = np.eye(self.dof) - (J_pinv @ J)
        q_dot_null = null_projector @ (self.k_null * (self.q_rest - q_current))

        # Συνολική ταχύτητα αρθρώσεων
        return q_dot_task + q_dot_null


# --- Αυτόνομη Δοκιμή (Standalone Test) ---
if __name__ == "__main__":
    solver = KinematicsEngine7DOF()
    q_test = np.zeros(7)
    v_test = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.1])  # Δοκιμαστική ταχύτητα

    p_end, J = solver.compute_fk_and_jacobian(q_test)
    q_dot = solver.solve_dls_ik(J, v_test, q_test)

    print("--- TESTING KINEMATICS ENGINE ---")
    print(f"End-Effector Pose (XYZ): {p_end.round(3)} m")
    pretty(J,"Jacobian")
    print(f"Calculated Joint Velocities (q_dot): {q_dot.round(3)}")
