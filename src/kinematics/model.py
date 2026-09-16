import numpy as np


class MyArmKinematics:

  def __init__(self, tool_length=0.045):
    # Nominal link offsets (σε μέτρα)
    d1 = 0.130
    d3 = 0.110
    d5 = 0.096
    d7 = 0.055 + tool_length  # Φλάντζα + vacuum gripper

    # DH Parameters: [a, alpha, d, theta_offset]
    self.dh_table = [
        [0.0, np.pi / 2, d1, 0.0],
        [0.0, np.pi / 2, 0.0, np.pi / 2],
        [0.0, -np.pi / 2, d3, 0.0],
        [0.0, np.pi / 2, 0.0, 0.0],
        [0.0, -np.pi / 2, d5, 0.0],
        [0.0, np.pi / 2, 0.0, 0.0],
        [0.0, 0.0, d7, 0.0],
    ]

    # Όρια γωνιών (σε ακτίνια) για το MyArm 300 Pi
    self.joint_limits_lower = np.radians(
        [-165, -165, -165, -165, -165, -165, -175]
    )
    self.joint_limits_upper = np.radians(
        [165, 165, 165, 165, 165, 165, 175]
    )

  @staticmethod
  def _dh_matrix(a, alpha, d, theta):
    """Υπολογίζει τον πίνακα μετασχηματισμού 4x4 για Standard DH."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])

  def forward_kinematics(self, q):
    """Επιστρέφει όλους τους ενδιάμεσους ομογενείς μετασχηματισμούς T_0_i

    και τον τελικό μετασχηματισμό T_0_e.
    q: iterable 7 γωνιών σε radians.
    """
    if len(q) != 7:
      raise ValueError(f"Απαιτούνται 7 joint angles, δόθηκαν {len(q)}")

    T = np.eye(4)
    transforms = [T]

    for i in range(7):
      a, alpha, d, theta_offset = self.dh_table[i]
      theta = q[i] + theta_offset
      A_i = self._dh_matrix(a, alpha, d, theta)
      T = T @ A_i
      transforms.append(T)

    return transforms

  def get_end_effector_pose(self, q):
    """Επιστρέφει τη θέση (x, y, z) και τον πίνακα περιστροφής R (3x3)."""
    transforms = self.forward_kinematics(q)
    T_0_e = transforms[-1]
    position = T_0_e[0:3, 3]
    rotation = T_0_e[0:3, 0:3]
    return position, rotation