import RPi.GPIO as GPIO
import time

# Ορισμός των δύο GPIO pins
PIN_1 = 20
PIN_2 = 21

# Αρχικοποίηση GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(PIN_1, GPIO.OUT)
GPIO.setup(PIN_2, GPIO.OUT)

# Ορισμός των 4 συνδυασμών
combinations = [
    ("LOW",  "HIGH", GPIO.LOW,  GPIO.HIGH),
    ("HIGH", "LOW",  GPIO.HIGH, GPIO.LOW),
    ("LOW",  "LOW",  GPIO.LOW,  GPIO.LOW),
    ("HIGH", "HIGH", GPIO.HIGH, GPIO.HIGH)
]

print("=== ΕΝΑΡΞΗ ΔΟΚΙΜΗΣ ΣΥΝΔΥΑΣΜΩΝ GPIO 20 & 21 ===")
print("Σημειώστε σε ποιον συνδυασμό ΣΤΑΜΑΤΑΕΙ ή ΞΕΚΙΝΑΕΙ η αντλία!\n")

try:
    for name1, name2, val1, val2 in combinations:
        print(f"----------------------------------------")
        print(f"[ΔΟΚΙΜΗ] GPIO 20 = {name1} | GPIO 21 = {name2}")
        print(f"----------------------------------------")
        
        GPIO.output(PIN_1, val1)
        GPIO.output(PIN_2, val2)
        
        # Αναμονή 3 δευτερολέπτων για να παρατηρήσετε την αντλία
        time.sleep(3.0)

except KeyboardInterrupt:
    print("\nΔιακοπή από τον χρήστη.")

finally:
    # Καθαρισμός των GPIO κατά την έξοδο
    GPIO.output(PIN_1, GPIO.HIGH)
    GPIO.output(PIN_2, GPIO.HIGH)
    GPIO.cleanup()
    print("\n=== Η ΔΟΚΙΜΗ ΟΛΟΚΛΗΡΩΘΗΚΕ (GPIO Cleanup) ===")
