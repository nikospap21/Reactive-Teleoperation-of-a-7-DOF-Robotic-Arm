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
    self.dof = 7
    self.lambda_max = lambda_max
    self.k_null = k_null
    self.max_joint_vel = max_joint_vel
    self.singularity_threshold = singularity_threshold

    # True για Revolute (1, 3, 5, 7), False για Prismatic (2, 4, 6)
    # Δείκτες 0-based: 0, 2, 4, 6 -> R | 1, 3, 5 -> P
    self.is_revolute = [True, False, True, False, True, False, True]

    # DH Parameters βάσης: [a, alpha, d, theta]
    self.dh_params = [
        [0.0, 0.0, 0.1405, 0.0],  # Joint 1: Revolute (θ1)
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 2: Prismatic (d2)
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 3: Revolute (θ3)
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 4: Prismatic (d4)
        [0.0, math.pi / 2, 0.1200, 0.0],  # Joint 5: Revolute (θ5)
        [0.0, -math.pi / 2, 0.0, 0.0],  # Joint 6: Prismatic (d6)
        [0.0, math.pi / 2, 0.0600, 0.0],  # Joint 7: Revolute (θ7)
    ]

    # Όρια: rad για 1,3,5,7 και μέτρα (m) για 2,4,6
    self.q_min = np.array([-2.79, 0.02, -2.79, 0.02, -2.79, 0.02, -2.79])
    self.q_max = np.array([2.79, 0.20, 2.79, 0.20, 2.79, 0.20, 2.79])
    self.q_mid = (self.q_max + self.q_min) / 2.0
    self.q_range = self.q_max - self.q_min

    self.q_rest = np.copy(self.q_mid)

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
    T = np.eye(4)
    positions = [T[0:3, 3]]
    z_axes = [T[0:3, 2]]

    for i in range(self.dof):
      a, alpha, d_base, theta_base = self.dh_params[i]

      if self.is_revolute[i]:
        theta = q[i] + theta_base
        d = d_base
      else:
        theta = theta_base
        d = q[i]  # Εδώ η μεταβλητή άρθρωσης είναι μετατόπιση

      T_i = self._dh_matrix(a, alpha, d, theta)
      T = T @ T_i
      positions.append(T[0:3, 3])
      z_axes.append(T[0:3, 2])

    p_end = positions[-1]
    R_end = T[0:3, 0:3]

    # Υπολογισμός Jacobian με βάση τον τύπο κάθε άρθρωσης
    J = np.zeros((6, self.dof))
    for i in range(self.dof):
      z_prev = z_axes[i]
      p_prev = positions[i]

      if self.is_revolute[i]:
        J[0:3, i] = np.cross(z_prev, p_end - p_prev)  # Linear από περιστροφή
        J[3:6, i] = z_prev  # Angular
      else:
        J[0:3, i] = z_prev  # Καθαρή γραμμική κίνηση κατά z_{i-1}
        J[3:6, i] = np.zeros(3)  # Μηδενική γωνιακή ταχύτητα

    return p_end, R_end, J

  def solve_dls_ik(self, J, v_cmd, q_current):
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

    # Null-space joint limit avoidance
    grad_limits = (q_current - self.q_mid) / (self.q_range**2)
    q_null_target = -1.5 * grad_limits + self.k_null * (
        self.q_rest - q_current
    )

    I_7 = np.eye(self.dof)
    null_projector = I_7 - (J_pinv @ J)
    q_dot_null = null_projector @ q_null_target

    q_dot = q_dot_task + q_dot_null

    max_val = np.max(np.abs(q_dot))
    if max_val > self.max_joint_vel:
      q_dot = q_dot * (self.max_joint_vel / max_val)

    return q_dot
