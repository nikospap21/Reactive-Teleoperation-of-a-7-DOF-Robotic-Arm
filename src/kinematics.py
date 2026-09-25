import math
import numpy as np


class KinematicsEngine7DOF:

    def __init__(
        self,
        lambda_max=0.04,
        k_null=0.1,
        max_joint_vel=1.5,
        singularity_threshold=0.02,
    ):
        """Ακριβής Κινηματική Μηχανή βασισμένη στο Επίσημο URDF του MyArm 300 Pi."""
        self.dof = 7
        self.lambda_max = lambda_max
        self.k_null = k_null
        self.max_joint_vel = max_joint_vel
        self.singularity_threshold = singularity_threshold

        # Ακριβή Transformations από το URDF (Origin XYZ, RPY για κάθε joint)
        # [x, y, z, roll, pitch, yaw]
        self.joint_origins = [
            [0.0, 0.0, 0.165, 0.0, 0.0, 0.0],           # Joint 1 -> Base
            [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0],         # Joint 2 -> Joint 1
            [0.0, -0.110, 0.0, 1.5708, 0.0, 0.0],       # Joint 3 -> Joint 2
            [0.0, 0.0, 0.0, -1.5708, 0.0, 3.14159],     # Joint 4 -> Joint 3
            [0.0, -0.126, 0.0, 1.5708, 0.0, 0.0],       # Joint 5 -> Joint 4
            [0.0, 0.0, 0.0, -1.5708, 0.0, 0.0],         # Joint 6 -> Joint 5
            [0.0, -0.056, 0.0, 1.5708, 0.0, 0.0],       # Joint 7 -> Joint 6
        ]

        # Rest pose εντός των πραγματικών ορίων του URDF
        self.q_rest = np.array([0.0, 0.5, 0.0, -0.5, 0.0, 0.3, 0.0])

    def _rpy_to_mat(self, r, p, y):
        """Μετατροπές Roll-Pitch-Yaw σε 3x3 Rotation Matrix."""
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        return R_z @ R_y @ R_x

    def _get_joint_transform(self, joint_idx, q_val):
        """Υπολογίζει τον 4x4 πίνακα μετασχηματισμού για κάθε joint βάσει URDF."""
        x, y, z, r, p, yaw = self.joint_origins[joint_idx]
        
        # Origin transformation
        T_origin = np.eye(4)
        T_origin[0:3, 0:3] = self._rpy_to_mat(r, p, yaw)
        T_origin[0:3, 3] = [x, y, z]

        # Rotation γύρω από τον άξονα Z κατά q_val
        cq, sq = math.cos(q_val), math.sin(q_val)
        T_rot = np.array([
            [cq, -sq, 0.0, 0.0],
            [sq,  cq, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        return T_origin @ T_rot

    def compute_fk_and_jacobian(self, q):
        """Υπολογισμός FK και 6x7 Geometric Jacobian απευθείας από τη URDF δομή."""
        T = np.eye(4)
        positions = [np.zeros(3)]
        z_axes = [np.array([0.0, 0.0, 1.0])]

        for i in range(self.dof):
            T_i = self._get_joint_transform(i, q[i])
            T = T @ T_i
            positions.append(T[0:3, 3])
            z_axes.append(T[0:3, 2])

        p_end = positions[-1]
        R_end = T[0:3, 0:3]
        J = np.zeros((6, self.dof))

        for i in range(self.dof):
            z_i = z_axes[i]
            p_i = positions[i]
            J[0:3, i] = np.cross(z_i, p_end - p_i)  # Linear Velocity (v)
            J[3:6, i] = z_i                        # Angular Velocity (w)

        return p_end, J

    def solve_dls_ik(self, J, v_cmd, q_current):
        """DLS IK Solver με Null-space projection."""
        JJt = J @ J.T
        det_JJt = np.linalg.det(JJt)
        w = math.sqrt(max(0.0, det_JJt))

        if w < self.singularity_threshold:
            damping = self.lambda_max * (1.0 - (w / self.singularity_threshold) ** 2)
        else:
            damping = 0.0

        damp_mat = (damping**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJt + damp_mat)

        q_dot_task = J_pinv @ v_cmd
        
        q_null_target = self.k_null * (self.q_rest - q_current)

        I_7 = np.eye(self.dof)
        null_projector = I_7 - (J_pinv @ J)
        q_dot_null = null_projector @ q_null_target

        q_dot = q_dot_task + q_dot_null

        max_val = np.max(np.abs(q_dot))
        if max_val > self.max_joint_vel:
            q_dot = q_dot * (self.max_joint_vel / max_val)

        return q_dot
