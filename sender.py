import socket
import json
import time
import sys
import pygame

# Ρύθμιση αποστολής στο WSL2 (το localhost 127.0.0.1 περνάει αυτόματα)
TARGET_IP = "172.25.119.136"
TARGET_PORT = 5005
UDP_ADDR = (TARGET_IP, TARGET_PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ Δεν βρέθηκε χειριστήριο! Σύνδεσε το USB dongle και ξαναπροσπάθησε.")
    sys.exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"✅ Συνδέθηκε επιτυχώς: {joystick.get_name()}")
print(f"Αριθμός Αξόνων: {joystick.get_numaxes()} | Κουμπιών: {joystick.get_numbuttons()}")
print(f"🚀 Αποστολή δεδομένων στο WSL2 ({TARGET_IP}:{TARGET_PORT})... (Ctrl+C για έξοδο)\n")

RATE_HZ = 50
DEADZONE = 0.08

try:
    while True:
        pygame.event.pump()

        # Ανάγνωση αξόνων (μοχλοί)
        axes = []
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            axes.append(0.0 if abs(val) < DEADZONE else round(val, 3))

        # Ανάγνωση κουμπιών
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        payload = {
            "axes": axes,
            "buttons": buttons,
            "timestamp": round(time.time(), 4)
        }

        # Αποστολή μέσω UDP
        sock.sendto(json.dumps(payload).encode('utf-8'), UDP_ADDR)
        time.sleep(1.0 / RATE_HZ)

except KeyboardInterrupt:
    print("\nΤερματισμός bridge.")
    pygame.quit()