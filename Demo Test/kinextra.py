import math
import numpy as np


class KinematicsEngine7DOF:

  def __init__(
      self,
      lambda_max=0.04,
      k_null=1.2,
      max_joint_vel=1.2,
      singularity_threshold=0.015,
  ):
    """Κινηματική μηχανή για τον βραχίονα Elephant Robotics MyArm 300 Pi (7-DOF).

    Υπολογίζει Forward Kinematics, Γεωμετρικό Ιακωβιανό (6x7), DLS Inverse
    Kinematics με Null-Space Optimization για αποφυγή ορίων και ομαλή κίνηση
    σε 6 διαστάσεις (XYZ + Roll/Pitch/Yaw).
    """
    # Standard DH Parameters: [a_i, alpha_i, d_i, theta_offset]
    self.dh_params = [
        [0.0, 0.0, 0.1405, 0.0],  # Joint 1: Base Yaw
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 2: Shoulder Pitch
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 3: Elbow Roll
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 4: Elbow Pitch
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 5: Wrist Roll
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 6: Wrist Pitch
        [0.0, math.pi / 2, 0.0600, 0.0],  # Joint 7: Wrist Yaw / Tool Flange
    ]

    self.dof = 7
    self.lambda_max = lambda_max
    self.k_null = k_null
    self.max_joint_vel = max_joint_vel
    self.singularity_threshold = singularity_threshold

    # Όρια γωνιών (σε ακτίνια)
    self.q_min = np.array([-2.79, -1.57, -2.79, -1.57, -2.79, -1.57, -2.79])
    self.q_max = np.array([2.79, 1.57, 2.79, 1.57, 2.79, 1.57, 2.79])
    self.q_mid = (self.q_max + self.q_min) / 2.0
    self.q_range = self.q_max - self.q_min

    # Ουδέτερη φυσική στάση (Home rest posture)
    self.q_rest = np.array([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0])

  def _dh_matrix(self, a, alpha, d, theta):
    """4x4 Standard DH Transformation Matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])

  def compute_fk_and_jacobian(self, q):
    """Υπολογίζει:

    - p_end: [x, y, z] του End-Effector στο Base frame
    - R_end: 3x3 πίνακας προσανατολισμού του End-Effector
    - J: 6x7 Γεωμετρικός Ιακωβιανός
    """
    T = np.eye(4)
    positions = [T[0:3, 3]]
    z_axes = [T[0:3, 2]]  # z0 = [0, 0, 1]

    for i in range(self.dof):
      a, alpha, d, theta_offset = self.dh_params[i]
      theta = q[i] + theta_offset
      T_i = self._dh_matrix(a, alpha, d, theta)
      T = T @ T_i
      positions.append(T[0:3, 3])
      z_axes.append(T[0:3, 2])

    p_end = positions[-1]
    R_end = T[0:3, 0:3]

    # Κατασκευή Γεωμετρικού Ιακωβιανού (6x7)
    J = np.zeros((6, self.dof))
    for i in range(self.dof):
      z_prev = z_axes[i]  # Άξονας z_{i-1}
      p_prev = positions[i]  # Θέση αρχής p_{i-1}
      J[0:3, i] = np.cross(z_prev, p_end - p_prev)  # Γραμμική συνιστώσα
      J[3:6, i] = z_prev  # Γωνιακή συνιστώσα

    return p_end, R_end, J

  def transform_v_to_base(self, v_tool, R_end):
    """Μετατρέπει ένα διάνυσμα ταχύτητας 6D από το σύστημα του End-Effector

    στο σύστημα της Βάσης.
    """
    v_base = np.zeros(6)
    v_base[0:3] = R_end @ v_tool[0:3]
    v_base[3:6] = R_end @ v_tool[3:6]
    return v_base

  def solve_dls_ik(self, J, v_cmd, q_current):
    """Επίλυση διαφορικής κινηματικής με:

    1. Damped Least Squares (αποφυγή ιδιομορφιών)
    2. Null-Space Optimization (ταυτόχρονη αποφυγή joint limits & άνετη στάση)
    """
    # 1. Έλεγχος Manipulability (Yoshikawa): w = sqrt(det(J * J^T))
    JJt = J @ J.T
    det_JJt = np.linalg.det(JJt)
    w = math.sqrt(max(0.0, det_JJt))

    # Προσαρμοσμένο Damping
    if w < self.singularity_threshold:
      damping = self.lambda_max * (1.0 - (w / self.singularity_threshold) ** 2)
    else:
      damping = 0.0

    # DLS Inverse: J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
    damp_mat = (damping**2) * np.eye(6)
    J_pinv = J.T @ np.linalg.inv(JJt + damp_mat)

    # 2. Κύρια κίνηση στο Καρτεσιανό επίπεδο (Task velocity)
    q_dot_task = J_pinv @ v_cmd

    # 3. Βελτιστοποίηση στον Μηδενικό Χώρο (Null-Space Projection)
    # Συνδυασμός: Αποφυγή ορίων αρθρώσεων + Έλξη προς το Home Posture
    grad_limits = (q_current - self.q_mid) / (self.q_range**2)
    q_null_target = -1.5 * grad_limits + self.k_null * (
        self.q_rest - q_current
    )

    I_7 = np.eye(self.dof)
    null_projector = I_7 - (J_pinv @ J)
    q_dot_null = null_projector @ q_null_target

    # 4. Τελική ταχύτητα
    q_dot = q_dot_task + q_dot_null

    # 5. Ομαλό scaling αν ξεπεραστεί το μέγιστο όριο
    max_val = np.max(np.abs(q_dot))
    if max_val > self.max_joint_vel:
      q_dot = q_dot * (self.max_joint_vel / max_val)

    return q_dot
    
