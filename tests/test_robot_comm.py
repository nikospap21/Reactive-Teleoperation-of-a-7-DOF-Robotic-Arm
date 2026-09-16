import os
import sys
import time

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)
from src.control.robot_interface import MyArmInterface


def main():
  # Δοκιμή σύνδεσης (βάλτε τη σωστή θύρα του συστήματός σας)
  robot = MyArmInterface(port="/dev/ttyAMA0", mock=False)

  q = robot.get_joint_angles()
  print(f"[+] Τρέχουσες γωνίες ρομπότ (rad): {q}")

  print("[*] Μετάβαση στην Home θέση...")
  robot.go_home(speed=30)

  print("[*] Ακινητοποίηση...")
  robot.stop_motion()
  print("[+] Test ολοκληρώθηκε επιτυχώς.")


if __name__ == "__main__":
  main()