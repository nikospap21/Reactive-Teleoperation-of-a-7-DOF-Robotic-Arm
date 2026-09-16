import os
import sys
import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from src.kinematics.redundancy_solver import RedundancySolver


def main():
  solver = RedundancySolver()

  # Αρχική διαμόρφωση όπου κάποιες αρθρώσεις είναι εκτός κέντρου
  q_current = np.radians([60.0, -45.0, 30.0, -60.0, 45.0, 10.0, -10.0])

  # Ζητάμε καθαρή γραμμική ταχύτητα κατά Z (vx=0, vy=0, vz=0.05 m/s, wx=0, wy=0, wz=0)
  v_desired = np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0])

  q_dot = solver.solve(q_current, v_desired)

  print("=== Redundancy Solver Test ===")
  print(f"Target Cartesian Vel v_e : {v_desired}")
  print(f"Resulting Joint Velocities q_dot (rad/s):\n{np.round(q_dot, 4)}")

  # Επαλήθευση: Αν πολλαπλασιάσουμε J * q_dot, πρέπει να πάρουμε σχεδόν ακριβώς το v_desired
  J = solver.jacobian_solver.compute_jacobian(q_current)
  v_achieved = J @ q_dot
  print(f"\nAchieved Cartesian Vel (J * q_dot):\n{np.round(v_achieved, 4)}")
  print(f"Error norm: {np.linalg.norm(v_desired - v_achieved):.6e}")


if __name__ == "__main__":
  main()