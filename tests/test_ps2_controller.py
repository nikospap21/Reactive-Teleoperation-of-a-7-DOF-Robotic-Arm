import os
import sys
import time

# Εξασφάλιση ότι το root directory βρίσκεται στο path
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
)

from src.input.ps2_controller import PS2Receiver


def main():
  print("Εκκίνηση PS2 Receiver Test...")
  receiver = PS2Receiver("config/controller_config.yaml")
  print(f"Receiver: {receiver.ip}:{receiver.port}")

  rate = 1.0 / receiver.cfg["controller"]["update_rate_hz"]

  try:
    while True:
      vx, vy, vz, wz, gripper, has_data = receiver.update()

      status = "DATA OK" if has_data else "WAITING"
      grip_str = "ON" if gripper else "OFF"

      print(
          f"\r[{status}] vx: {vx:+.3f} | vy: {vy:+.3f} | vz: {vz:+.3f} | wz:"
          f" {wz:+.3f} | Gripper: [{grip_str}]",
          end="",
          flush=True,
      )

      time.sleep(rate)

  except KeyboardInterrupt:
    print("\n[*] Τερματισμός δοκιμής.")


if __name__ == "__main__":
  main()