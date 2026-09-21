import time
from pymycobot import MyArm

arm = MyArm("/dev/ttyAMA0", 115200)
time.sleep(0.5)

print("--- ΤΕΣΤ ΑΝΤΛΙΑΣ ΚΕΝΟΥ (PIN 15) ---")
print("1. Ενεργοποίηση Αντλίας (GRASP)...")
arm.set_basic_output(15, 0)  # Relay ON
time.sleep(3)

print("2. Απενεργοποίηση Αντλίας (RELEASE)...")
arm.set_basic_output(15, 1)  # Relay OFF
time.sleep(1)

print("Τεστ Ολοκληρώθηκε!")
