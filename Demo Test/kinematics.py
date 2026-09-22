import math
import numpy as np


class KinematicsEngine7DOF:

  def __init__(
      self,
      lambda_max=0.04,
      k_null=0.2,
      max_joint_vel=0.8,
      singularity_threshold=0.015,
  ):
    """Κινηματική Μηχανή για Elephant Robotics MyArm 300 Pi (7-DOF).

    Υποστηρίζει Dynamic Damped Least Squares (DLS), Redundancy Resolution μέσω
    Null-Space, και Scaling ταχυτήτων για ομαλή κίνηση (Smoothness).
    """
    # Standard DH Parameters [a, alpha, d, theta_offset]
    self.dh_params = [
        [0.0, 0.0, 0.1405, 0.0],  # Joint 1
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 2
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 3
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 4
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 5
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 6
        [0.0, math.pi / 2, 0.0600, 0.0],  # Joint 7 (End-Effector)
    ]

    self.dof = 7
    self.lambda_max = lambda_max
    self.k_null = k_null
    self.max_joint_vel = max_joint_vel  # rad/s max ταχύτητα για εξομάλυνση
    self.singularity_threshold = singularity_threshold

    # Φυσική, εργονομική στάση ανάπαυσης (μακριά από joint limits)
    self.q_rest = np.radians([0.0, 30.0, 0.0, 60.0, 0.0, 30.0, 0.0])

  def _dh_matrix(self, a, alpha, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])

  def compute_fk_and_jacobian(self, q):
    """Υπολογισμός Forward Kinematics (θέση άκρου) και Γεωμετρικού Ιακωβιανού (6x7)."""
    T = np.eye(4)
    positions = [np.zeros(3)]
    z_axes = [np.array([0.0, 0.0, 1.0])]

    for i in range(self.dof):
      a, alpha, d, theta_offset = self.dh_params[i]
      theta = q[i] + theta_offset
      T_i = self._dh_matrix(a, alpha, d, theta)
      T = T @ T_i
      positions.append(T[0:3, 3])
      z_axes.append(T[0:3, 2])

    p_end = positions[-1]
    R_end = T[0:3, 0:3]  # Ο πίνακας περιστροφής του End-Effector
    J = np.zeros((6, self.dof))

    for i in range(self.dof):
      z_i = z_axes[i]
      p_i = positions[i]
      J[0:3, i] = np.cross(z_i, p_end - p_i)  # Linear
      J[3:6, i] = z_i  # Angular

    return p_end, J

  # def transform_v_to_base(self, v_cmd_tool, R_end):
  #       """
  #       Μετατρέπει τις ταχύτητες από το Tool Frame στο Base Frame 
  #       ώστε ο χειρισμός να είναι απόλυτα διαισθητικός (Intuitive).
  #       """
  #       v_lin_base = R_end @ v_cmd_tool[0:3]
  #       v_ang_base = R_end @ v_cmd_tool[3:6]
  #       return np.concatenate([v_lin_base, v_ang_base])

  def solve_dls_ik(self, J, v_cmd, q_current):
    """Υπολογισμός ταχυτήτων αρθρώσεων με:

    1. Dynamic Damping DLS (αποφυγή εκρήξεων σε ιδιομορφίες) 2. Null-Space
    Optimization (άνετη στάση, αποφυγή τρανταγμάτων) 3. Joint Velocity
    Normalization (smooth motion)
    """
    # 1. Υπολογισμός Manipulability Measure: w = sqrt(det(J * J^T))
    JJt = J @ J.T
    det_JJt = np.linalg.det(JJt)
    w = math.sqrt(max(0.0, det_JJt))

    # 2. Προσαρμοζόμενος Συντελεστής Απόσβεσης (Dynamic Damping)
    # Όσο πλησιάζουμε σε ιδιομορφία (w -> 0), τόσο αυξάνεται το damping για να μην τιναχτεί ο βραχίονας
    if w < self.singularity_threshold:
      damping = self.lambda_max * (1.0 - (w / self.singularity_threshold) ** 2)
    else:
      damping = 0.0

    # DLS Inverse: J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
    damp_mat = (damping**2) * np.eye(6)
    J_pinv = J.T @ np.linalg.inv(JJt + damp_mat)

    # 3. Task Velocity
    q_dot_task = J_pinv @ v_cmd

    # 4. Null-Space Redundancy Resolution: (I - J_pinv * J) * q_null
    # Σπρώχνει τις αρθρώσεις ομαλά προς την άνετη στάση χωρίς να επηρεάζει το άκρο
    I_7 = np.eye(self.dof)
    null_projector = I_7 - (J_pinv @ J)
    q_null_target = self.k_null * (self.q_rest - q_current)
    q_dot_null = null_projector @ q_null_target

    # Συνολική ταχύτητα
    q_dot = q_dot_task + q_dot_null

    # 5. Smooth Speed Limiting (Αποφυγή τραντάγματος - scaling διανύσματος)
    max_val = np.max(np.abs(q_dot))
    if max_val > self.max_joint_vel:
      q_dot = q_dot * (self.max_joint_vel / max_val)

    return q_dot
