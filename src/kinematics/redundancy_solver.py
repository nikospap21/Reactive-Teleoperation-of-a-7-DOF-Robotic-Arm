import numpy as np
from src.kinematics.jacobian import MyArmJacobian
from src.kinematics.model import MyArmKinematics


class RedundancySolver:

  def __init__(
      self,
      kinematics: MyArmKinematics = None,
      jacobian_solver: MyArmJacobian = None,
      damping=0.05,
      k_null=1.5,
  ):
    self.kinematics = kinematics if kinematics else MyArmKinematics()
    self.jacobian_solver = (
        jacobian_solver if jacobian_solver else MyArmJacobian(self.kinematics)
    )

    self.damping = damping  # lambda για DLS
    self.k_null = k_null  # Κέρδος για null-space posture optimization

    self.q_min = self.kinematics.joint_limits_lower
    self.q_max = self.kinematics.joint_limits_upper
    self.q_mid = (self.q_max + self.q_min) / 2.0
    self.q_range = self.q_max - self.q_min

  def _compute_dls_pseudoinverse(self, J):
    """Υπολογίζει το Damped Least Squares ψευδοαντίστροφο: J^T * (J*J^T + lambda^2 * I)^(-1)."""
    m, n = J.shape  # 6 x 7
    damping_matrix = (self.damping**2) * np.eye(m)
    # J * J^T είναι 6x6
    inv_term = np.linalg.inv(J @ J.T + damping_matrix)
    J_dls = J.T @ inv_term
    return J_dls

  def _null_space_gradient(self, q):
    """Υπολογίζει το - grad(H(q)) που επαναφέρει τις αρθρώσεις προς το κέντρο του εύρους τους."""
    grad = (q - self.q_mid) / (self.q_range**2)
    return -self.k_null * grad

  def solve(self, q, v_cartesian):
    """Επιλύει τη διαφορική κινηματική με διαχείριση redundancy και αποφυγή ορίων.

    q: Τρέχουσα διαμόρφωση (7 elements array)
    v_cartesian: Επιθυμητή καρτεσιανή ταχύτητα [vx, vy, vz, wx, wy, wz]^T (6 elements)
    Επιστρέφει: q_dot (7 elements array)
    """
    q = np.asarray(q, dtype=float)
    v = np.asarray(v_cartesian, dtype=float)

    # 1. Υπολογισμός Ιακωβιανού J(q)
    J = self.jacobian_solver.compute_jacobian(q)

    # 2. DLS Inverse J^dagger
    J_pinv = self._compute_dls_pseudoinverse(J)

    # 3. Πρωτεύουσα ταχύτητα (End-effector motion)
    q_dot_primary = J_pinv @ v

    # 4. Null-Space Projector: P = I - J^dagger * J
    P_null = np.eye(7) - (J_pinv @ J)

    # 5. Δευτερεύων στόχος: Αποφυγή ορίων αρθρώσεων
    q_dot_0 = self._null_space_gradient(q)

    # 6. Συνολική ταχύτητα
    q_dot = q_dot_primary + (P_null @ q_dot_0)

    return q_dot