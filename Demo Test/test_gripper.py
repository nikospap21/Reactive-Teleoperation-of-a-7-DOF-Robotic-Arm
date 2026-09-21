import RPi.GPIO as GPIO
import time

PINS = [16, 20]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

print("--- ΔΟΚΙΜΗ GPIO 16 & GPIO 20 ---")

for pin in PINS:
    print(f"\n[ TESTING GPIO {pin} ]")
    GPIO.setup(pin, GPIO.OUT)
    
    print(f" -> GPIO {pin} = LOW (0) [ON]")
    GPIO.output(pin, GPIO.LOW)
    time.sleep(2)
    
    print(f" -> GPIO {pin} = HIGH (1) [OFF]")
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(1)

GPIO.cleanup()
print("\nΔοκιμή Ολοκληρώθηκε!")
