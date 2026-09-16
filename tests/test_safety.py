import os
import sys
import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from src.control.safety import SafetySupervisor


def main():
  safety = SafetySupervisor(max_joint_vel=0.8, buffer_zone_deg=5.0)

  # Σενάριο 1: Η άρθρωση 1 είναι κολλημένη στο άνω όριο (+164 μοίρες)
  q_current = np.zeros(7)
  q_current[0] = np.radians(164.0)  # Μέσα στη ζώνη buffer

  # Ο χρήστης δίνει ταχύτητα που πάει να το σπρώξει κι άλλο προς τα πάνω
  q_dot_unsafe = np.array([0.5, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
  q_dot_filtered = safety.enforce_joint_limits(q_current, q_dot_unsafe)

  print("=== Test 1: Upper Limit Blocking ===")
  print(f"q[0] = {np.degrees(q_current[0]):.1f} deg (Upper limit: 165.0)")
  print(f"Requested q_dot[0]: {q_dot_unsafe[0]}")
  print(f"Filtered  q_dot[0]: {q_dot_filtered[0]} (Πρέπει να είναι 0.0)")

  # Σενάριο 2: Ο χρήστης δίνει ταχύτητα που απομακρύνει την άρθρωση από το όριο
  q_dot_backwards = np.array([-0.5, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
  q_dot_allowed = safety.enforce_joint_limits(q_current, q_dot_backwards)
  print(f"\nBackwards q_dot[0]: {q_dot_backwards[0]}")
  print(
      f"Filtered  q_dot[0]: {q_dot_allowed[0]} (Πρέπει να επιτραπεί κανονικά)"
  )

  # Σενάριο 3: Υπερβολική ταχύτητα (Uniform scaling test)
  q_dot_overspeed = np.array([2.4, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
  q_dot_scaled = safety.scale_velocities(q_dot_overspeed)
  print("\n=== Test 2: Velocity Scaling ===")
  print(f"Original: {q_dot_overspeed}")
  print(f"Scaled  : {np.round(q_dot_scaled, 3)}")
  print(f"Ratio preserved: {q_dot_scaled[0] / q_dot_scaled[1]:.2f} == 3.00")


if __name__ == "__main__":
  main()