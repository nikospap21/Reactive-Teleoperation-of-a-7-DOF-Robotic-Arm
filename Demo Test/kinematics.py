import numpy as np

class KinematicsEngine7DOF:
    def __init__(self):
        # Damped Least Squares (DLS) Damping factor
        self.lambda_sq = 0.015
        
        # Όρια Αρθρώσεων MyArm 300 Pi (σε rad)
        self.q_min = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
        self.q_max = np.array([ 2.79,  1.57,  2.79,  1.57,  2.79,  1.57,  2.79])
        self.q_mid = (self.q_min + self.q_max) / 2.0
        
        # Gain για την απώθηση από τα όρια στο Null-Space
        self.k_null = 0.8  

    def compute_fk_and_jacobian(self, q):
        """
        Υπολογισμός Forward Kinematics και Geometric Jacobian (6x7).
        """
        # Απλοποιημένη προσέγγιση Geometric Jacobian για 7-DOF MyArm
        # (Χρησιμοποιεί DH/Transformations του MyArm 300)
        p_end = np.array([
            0.18 * np.cos(q[0]) * np.cos(q[1]) + 0.15 * np.cos(q[0]) * np.cos(q[1]+q[3]),
            0.18 * np.sin(q[0]) * np.cos(q[1]) + 0.15 * np.sin(q[0]) * np.cos(q[1]+q[3]),
            0.12 + 0.18 * np.sin(q[1]) + 0.15 * np.sin(q[1]+q[3])
        ])

        # Κατασκευή Jacobian 6x7
        J = np.zeros((6, 7))
        J[0, 0] = -p_end[1]
        J[1, 0] =  p_end[0]
        J[0, 1] = -0.18 * np.cos(q[0]) * np.sin(q[1])
        J[1, 1] = -0.18 * np.sin(q[0]) * np.sin(q[1])
        J[2, 1] =  0.18 * np.cos(q[1])
        
        J[0, 3] = -0.15 * np.cos(q[0]) * np.sin(q[1]+q[3])
        J[1, 3] = -0.15 * np.sin(q[0]) * np.sin(q[1]+q[3])
        J[2, 3] =  0.15 * np.cos(q[1]+q[3])
        
        # Axis Roll/Yaw contribution
        J[3:, 0] = [0, 0, 1]
        J[3:, 2] = [np.cos(q[0]), np.sin(q[0]), 0]
        J[3:, 4] = [np.cos(q[0]), np.sin(q[0]), 0]
        J[3:, 6] = [0, 0, 1]

        return p_end, J

    def _compute_joint_limit_gradient(self, q):
        """
        Υπολογισμός κλίσης (Gradient) που απωθεί τις αρθρώσεις από τα όρια.
        """
        # Κανονικοποιημένη απόσταση από το κέντρο
        range_sq = (self.q_max - self.q_min) ** 2
        grad = (q - self.q_mid) / range_sq
        return grad

    def solve_dls_ik(self, J, v_cmd, q_current):
        """
        Damped Least Squares IK με ΕΝΕΡΓΗ ΑΠΟΦΥΓΗ ΟΡΙΩΝ στο Null-Space.
        """
        # 1. Damped Pseudo-Inverse: J_damped = J^T * (J * J^T + lambda^2 * I)^-1
        JJT = J @ J.T
        damped_inv = np.linalg.inv(JJT + self.lambda_sq * np.eye(6))
        J_pinv = J.T @ damped_inv

        # 2. Κύρια Καρτεσιανή Κίνηση
        q_dot_cartesian = J_pinv @ v_cmd

        # 3. Null-Space Projection Matrix: N = (I - J_pinv * J)
        I7 = np.eye(7)
        N = I7 - (J_pinv @ J)

        # 4. Null-Space Task: Απώθηση από τα όρια των αρθρώσεων
        grad_limits = self._compute_joint_limit_gradient(q_current)
        q_dot_null = N @ (-self.k_null * grad_limits)

        # 5. Τελική Ταχύτητα Αρθρώσεων (Καρτεσιανή + Προστασία Ορίων)
        q_dot_total = q_dot_cartesian + q_dot_null

        return q_dot_total
