import time
from pymycobot import MyArm # type:ignore

print("--- ΔΙΑΓΝΩΣΤΙΚΟ TEST RELAY (PINS 16 & 19) ---")
try:
    arm = MyArm("/dev/ttyAMA0", 115200)
    time.sleep(0.5)

    print("\n1. Δοκιμή: Pin 16 LOW (0), Pin 19 HIGH (1)")
    arm.set_basic_output(16, 0)
    arm.set_basic_output(19, 1)
    time.sleep(2)

    print("2. Δοκιμή: Pin 16 HIGH (1), Pin 19 LOW (0)")
    arm.set_basic_output(16, 1)
    arm.set_basic_output(19, 0)
    time.sleep(2)

    print("3. Δοκιμή: ΚΑΙ ΤΑ ΔΥΟ LOW (0, 0)")
    arm.set_basic_output(16, 0)
    arm.set_basic_output(19, 0)
    time.sleep(2)

    print("4. Απενεργοποίηση: ΚΑΙ ΤΑ ΔΥΟ HIGH (1, 1)")
    arm.set_basic_output(16, 1)
    arm.set_basic_output(19, 1)

    print("\n[ΤΕΛΟΣ ΔΟΚΙΜΗΣ] Ακούστηκε κάποιο 'κλικ' ή δούλεψε η αντλία σε κάποιο βήμα;")

except Exception as e:
    print(f"Σφάλμα επικοινωνίας: {e}")
