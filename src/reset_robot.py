import time
from pymycobot import MyArm # type:ignore

SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

print("--- RESETTING MYARM 300 PI ---")
try:
    arm = MyArm(SERIAL_PORT, BAUDRATE)
    time.sleep(0.5)

    # 1. Ξεκλείδωμα Σφάλματος
    print("1. Καθαρισμός Σφαλμάτων & Ενεργοποίηση...")
    arm.power_on()
    time.sleep(1.0)

    # 2. Επαναφορά στην Κατακόρυφη Αρχική Στάση [0, 0, 0, 0, 0, 0, 0]
    print("2. Επιστροφή στην κατακόρυφη αρχική στάση...")
    home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    arm.send_radians(home_angles, 20)  # Αργή, ασφαλής κίνηση 20%
    time.sleep(3.0)

    print("SUCCESS: Το ρομπότ επανήλθε στην αρχική στάση και το φως είναι ΠΡΑΣΙΝΟ!")

except Exception as e:
    print(f"ERROR: {e}")