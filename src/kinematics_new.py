import math
import numpy as np


def skew(w):
  """Skew-symmetric matrix for a 3D vector w."""
  return np.array(
      [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]
  )


def twist_exp(S, theta):
  """Υπολογισμός Matrix Exponential e^([S]*theta) για ένα Screw Axis S = [w, v]."""
  w = S[:3]
  v = S[3:]

  if np.allclose(w, 0):
    R = np.eye(3)
    p = v * theta
  else:
    w_hat = skew(w)
    R = (
        np.eye(3)
        + np.sin(theta) * w_hat
        + (1.0 - np.cos(theta)) * (w_hat @ w_hat)
    )
    p = (
        (np.eye(3) * theta)
        + (1.0 - np.cos(theta)) * w_hat
        + (theta - np.sin(theta)) * (w_hat @ w_hat)
    ) @ v

  T = np.eye(4)
  T[:3, :3] = R
  T[:3, 3] = p
  return T


def adjoint(T):
  """Adjoint Transformation Matrix [Ad_T] 6x6."""
  R = T[:3, :3]
  p = T[:3, 3]
  adj = np.zeros((6, 6))
  adj[:3, :3] = R
  adj[3:, 3:] = R
  adj[3:, :3] = skew(p) @ R
  return adj


class KinematicsEngine7DOF:

  def __init__(
      self,
      lambda_max=0.02,
      k_null=0.15,
      max_joint_vel=1.5,
      singularity_threshold=0.005,
  ):
    """Product of Exponentials (PoE) Kinematics Engine για το MyArm 300 Pi."""
    self.dof = 7
    self.lambda_max = lambda_max
    self.k_null = k_null
    self.max_joint_vel = max_joint_vel
    self.singularity_threshold = singularity_threshold

    # EE Offset (0.15m)
    EE_Offset = 0.15

    # 1. Home Configuration Matrix M (4x4) στο zero pose (q = [0, ..., 0])
    # Υπολογισμένο με βάση τα αθροιστικά XYZ/RPY offsets του URDF
    self.M = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.056],  # Y offset
        [0.0, 0.0, 1.0, 0.165 + EE_Offset],  # Z offset
        [0.0, 0.0, 0.0, 1.0],
    ])

    # 2. Screw Axes S = [w, v]^T στο Base Frame
    # Υπολογισμένα ακριβώς από τα joint origins του URDF
    self.S = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],  # Joint 1
        [0.0, -1.0, 0.0, 0.165, 0.0, 0.0],  # Joint 2
        [0.0, 0.0, 1.0, -0.110, 0.0, 0.0],  # Joint 3
        [0.0, 1.0, 0.0, -0.165, 0.0, 0.0],  # Joint 4
        [0.0, 0.0, 1.0, -0.236, 0.0, 0.0],  # Joint 5
        [0.0, -1.0, 0.0, 0.165, 0.0, 0.0],  # Joint 6
        [0.0, 0.0, 1.0, -0.056, 0.0, 0.0],  # Joint 7
    ]).T  # Shape: 6x7

    # Rest pose & Joint limits
    self.q_rest = np.array([0.0, 0.5, 0.0, -0.5, 0.0, -0.3, 0.0])
    self.q_min = np.array(
        [-2.87, -1.22, -2.87, -1.97, -2.87, -1.74, -3.05], dtype=float
    )
    self.q_max = np.array(
        [2.87, 2.01, 2.87, 1.30, 2.87, 1.83, 3.05], dtype=float
    )

  def compute_fk_and_jacobian(self, q):
    """Product of Exponentials (PoE) Forward Kinematics & Space Jacobian."""
    T = np.eye(4)
    J = np.zeros((6, self.dof))

    # Υπολογισμός Space Jacobian & T(q) μέσω Matrix Exponentials
    for i in range(self.dof):
      J[:, i] = adjoint(T) @ self.S[:, i]
      T_i = twist_exp(self.S[:, i], q[i])
      T = T @ T_i

    # Τελικός ομογενής μετασχηματισμός End-Effector
    T_end = T @ self.M

    p_end = T_end[0:3, 3]  # Θέση XYZ
    R_end = T_end[0:3, 0:3]  # Rotation matrix End-Effector (για το Tool Frame)

    return p_end, R_end, J

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

    # 1. Task Space Command
    q_dot_task = J_pinv @ v_cmd

    # 2. Null-Space Projection (q_rest)
    q_null_target = self.k_null * (self.q_rest - q_current)
    I_7 = np.eye(self.dof)
    null_projector = I_7 - (J_pinv @ J)
    q_dot_null = null_projector @ q_null_target

    q_dot = q_dot_task + q_dot_null

    # 3. Scaling ταχυτήτων
    max_val = np.max(np.abs(q_dot))
    if max_val > self.max_joint_vel:
      q_dot = q_dot * (self.max_joint_vel / max_val)

    return q_dot
