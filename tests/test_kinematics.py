import os
import sys
import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from src.kinematics.jacobian import MyArmJacobian
from src.kinematics.model import MyArmKinematics


def main():
  robot = MyArmKinematics()
  jac_solver = MyArmJacobian(robot)

  # Τυχαία μη-εκφυλισμένη διαμόρφωση (π.χ. ελαφρώς λυγισμένος βραχίονας)
  q_test = np.radians([10.0, 20.0, -15.0, 30.0, 10.0, -20.0, 5.0])

  pos, rot = robot.get_end_effector_pose(q_test)
  J = jac_solver.compute_jacobian(q_test)

  print("=== Jacobian Matrix Test ===")
  print(f"Joint configuration (deg): {np.degrees(q_test)}")
  print(f"End-Effector Position (m): {np.round(pos, 4)}")
  print(f"Jacobian shape: {J.shape}")
  print("\nJacobian J(q):\n", np.round(J, 3))

  # Έλεγχος ιδιοτιμών (Singular Values)
  # Επειδή το J είναι 6x7, αναλύουμε σε singular values:
  _, s, _ = np.linalg.svd(J)
  print(f"\nSingular values: {np.round(s, 4)}")
  print(f"Manipulability measure (Yoshikawa): {np.prod(s):.6f}")


if __name__ == "__main__":
  main()