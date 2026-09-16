import numpy as np
from src.kinematics.model import MyArmKinematics


class MyArmJacobian:

  def __init__(self, kinematics_model: MyArmKinematics = None):
    if kinematics_model is None:
      self.kinematics = MyArmKinematics()
    else:
      self.kinematics = kinematics_model

  def compute_jacobian(self, q):
    """Υπολογίζει τον γεωμετρικό Ιακωβιανό πίνακα 6x7 στο base frame.

    q: array-like 7 γωνιών σε radians.
    Επιστρέφει: J μεγέθους (6, 7), όπου οι πρώτες 3 γραμμές αφορούν
    γραμμική ταχύτητα και οι επόμενες 3 γωνιακή ταχύτητα.
    """
    transforms = self.kinematics.forward_kinematics(q)

    # Θέση end-effector (frame 7)
    p_e = transforms[-1][0:3, 3]

    J = np.zeros((6, 7), dtype=float)

    for i in range(7):
      T_prev = transforms[i]  # T_0^{i-1}

      # Ο άξονας z_{i-1} είναι η τρίτη στήλη του R_0^{i-1}
      z_prev = T_prev[0:3, 2]

      # Η θέση της άρθρωσης p_{i-1}
      p_prev = T_prev[0:3, 3]

      # Γραμμική συνιστώσα: v = z_{i-1} x (p_e - p_{i-1})
      J_linear = np.cross(z_prev, (p_e - p_prev))

      # Γωνιακή συνιστώσα: w = z_{i-1}
      J_angular = z_prev

      J[0:3, i] = J_linear
      J[3:6, i] = J_angular

    return J