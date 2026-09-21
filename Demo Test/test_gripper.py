import RPi.GPIO as GPIO
import time

# Επιλογή Pin (Δοκιμάστε το 20, 26, 16, 21)
PIN = 20  # Αλλάξτε το σε 26 ή 16 αν χρειαστεί

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

print(f"--- ΔΟΚΙΜΗ GPIO {PIN} ---")
print("1. Ενεργοποίηση (Relay LOW)...")
GPIO.output(PIN, GPIO.LOW)   # Σχεδόν όλα τα relays ενεργοποιούνται με LOW (0)
time.sleep(2)

print("2. Απενεργοποίηση (Relay HIGH)...")
GPIO.output(PIN, GPIO.HIGH)  # Relay OFF
time.sleep(1)

GPIO.cleanup()
print("Δοκιμή ολοκληρώθηκε!")
