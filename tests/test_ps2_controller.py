import os
import sys
import time

# Προσθήκη του root directory στο path της Python
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
  sys.path.insert(0, ROOT_DIR)

from src.input.ps2_controller import DirectPS2Controller

def main():
  print("=" * 60)
  print("   Δοκιμή Χειριστηρίου PS2 (Απευθείας USB στο Raspberry Pi)   ")
  print("=" * 60)

  # Αρχικοποίηση του direct controller (φορτώνει αυτόματα το config)
  controller = DirectPS2Controller()

  if controller.joystick is None:
    print("[ERR] Δεν βρέθηκε χειριστήριο συνδεδεμένο στο Raspberry Pi.")
    print("      Συνδέστε το USB dongle/καλώδιο και ξαναδοκιμάστε.")
    return

  # Συχνότητα ανανέωσης από το config ή default 30 Hz
  update_hz = controller.config.get("controller", {}).get("update_rate_hz", 30)
  rate = 1.0 / update_hz

  print("[+] Εκκίνηση ανάγνωσης μοχλών και πλήκτρων...")
  print("    Πατήστε Ctrl+C για έξοδο.\n")

  try:
    while True:
      vx, vy, vz, wz, gripper, connected = controller.update()

      status = "ONLINE" if connected else "NO DEVICE"
      grip_str = "ON" if gripper else "OFF"

      # Real-time εκτύπωση στην ίδια γραμμή
      sys.stdout.write(
          f"\r[{status}] vx: {vx:+.3f} | vy: {vy:+.3f} | vz: {vz:+.3f} | wz:"
          f" {wz:+.3f} | Suction: [{grip_str}]"
      )
      sys.stdout.flush()

      time.sleep(rate)

  except KeyboardInterrupt:
    print("\n\n[*] Τερματισμός δοκιμής από τον χρήστη.")


if __name__ == "__main__":
  main()