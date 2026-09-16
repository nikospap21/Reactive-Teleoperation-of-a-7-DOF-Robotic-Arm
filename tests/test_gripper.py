import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.gripper.vacuum_gripper import VacuumGripper

def main():
    print("=== Testing Vacuum Gripper ===")
    gripper = VacuumGripper(mock=True) # Αλλάξτε σε mock=False στο Pi

    print("[1] Ενεργοποίηση αναρρόφησης (Grab)...")
    gripper.grab()
    time.sleep(3.0)

    print("[2] Απελευθέρωση (Release)...")
    gripper.release()
    time.sleep(1.0)

    gripper.cleanup()
    print("=== Gripper Test Completed ===")

if __name__ == "__main__":
    main()