import pygame
import numpy as np
import RPi.GPIO as GPIO

# Ορισμός των δύο GPIO pins ελέγχου
PIN_20 = 20
PIN_21 = 21

""" PS2 Controller Class for MyArm 300 Pi Teleoperation"""
class PS2Controller:
    def __init__(self, device_index=0, deadzone = 0.0):

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Σφάλμα: Το controller δεν είναι συνδεδεμένο")

        self.joystick = pygame.joystick.Joystick(device_index)
        self.joystick.init()
        
        self.deadzone = deadzone
        self.vacuum_state = False

        # Ταχύτητες κίνησης και περιστροφής 
        self.scale_pos = 0.05
        self.scale_pos_z = 0.05
        self.scale_rot = 0.05

        # Ρύθμιση GPIO για Relay
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_20, GPIO.OUT)
        GPIO.setup(PIN_21, GPIO.OUT)
        
        GPIO.output(PIN_20, GPIO.LOW)
        GPIO.output(PIN_21, GPIO.HIGH)

    # Εφαρμογή deadzone για μοχλούς και triggers
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    # Κανονικοποίηση Trigger από [-1, 1] σε [0, 1]
    def normalize_trigger(self, raw_value):
        norm_val = (raw_value + 1.0) / 2.0
        return self.apply_deadzone(norm_val)

    # Σύνδεση μοχλών και triggers σε 6D ταχύτητα (vx, vy, vz, wx, wy, wz)
    def get_cartesian_velocity(self):

        pygame.event.pump()

        num_axes = self.joystick.get_numaxes()
        num_buttons = self.joystick.get_numbuttons()

        # Τιμές μοχλών και triggers
        raw_lx = self.joystick.get_axis(0) if num_axes > 0 else 0.0
        raw_ly = self.joystick.get_axis(1) if num_axes > 1 else 0.0
        raw_lz_up = self.joystick.get_axis(5) if num_axes > 5 else -1.0
        raw_lz_down = self.joystick.get_axis(2) if num_axes > 2 else -1.0
        raw_rx = self.joystick.get_axis(4) if num_axes > 4 else 0.0
        raw_ry = self.joystick.get_axis(3) if num_axes > 3 else 0.0
        raw_rz_up = self.joystick.get_button(4) if num_buttons > 4 else 0.0
        raw_rz_down = self.joystick.get_button(5) if num_buttons > 5 else 0.0

        # Εφαρμογή deadzone και κανονικοποίηση
        lx = self.apply_deadzone(raw_lx)
        ly = self.apply_deadzone(raw_ly)
        lz_up = self.normalize_trigger(raw_lz_up)
        lz_down = self.normalize_trigger(raw_lz_down)
        rx = self.apply_deadzone(raw_rx)
        ry = self.apply_deadzone(raw_ry)
        rz_up = self.apply_deadzone(raw_rz_up)
        rz_down = self.apply_deadzone(raw_rz_down)

        # Υπολογισμός 6D ταχύτητας (vx, vy, vz, wx, wy, wz)
        vx = -lx * self.scale_pos
        vy = -ly * self.scale_pos
        vz = (lz_up - lz_down) * self.scale_pos_z
        
        wx = rx * self.scale_rot
        wy = ry * self.scale_rot      
        wz = (rz_up - rz_down) * self.scale_rot

        # Noise Filtering
        if abs(vx) < 0.02: vx = 0.0
        if abs(vy) < 0.02: vy = 0.0
        if abs(vz) < 0.02: vz = 0.0
        if abs(wx) < 0.02: wx = 0.0
        if abs(wy) < 0.02: wy = 0.0
        if abs(wz) < 0.02: wz = 0.0

        return np.array([vx, vy, vz, wx, wy, wz])

    # Vacuum gripper
    def process_gripper_events(self):
        pygame.event.pump()

        if self.joystick.get_numbuttons() > 0 and self.joystick.get_button(0):
            if not self.vacuum_state:
                self.vacuum_state = True
                GPIO.output(PIN_20, GPIO.HIGH)
                GPIO.output(PIN_21, GPIO.LOW)
                print("\n Grasp\n")

        elif self.joystick.get_numbuttons() > 1 and self.joystick.get_button(1):
            if self.vacuum_state:
                self.vacuum_state = False
                GPIO.output(PIN_20, GPIO.LOW)
                GPIO.output(PIN_21, GPIO.HIGH)
                print("\n Release\n")

        return self.vacuum_state

    # Reset
    def process_reset_event(self):

        pygame.event.pump()
        num_buttons = self.joystick.get_numbuttons()

        reset_button_indices = [2]

        for btn_idx in reset_button_indices:
            if num_buttons > btn_idx and self.joystick.get_button(btn_idx):
                print("\n Reset\n")
                return True

        return False

if __name__ == "__main__":
    import time
    ctrl = PS2Controller()
    print("--- control test ---")
    while True:
        v = ctrl.get_cartesian_velocity()
        if np.linalg.norm(v) == 0:
            print("Status: ΑΚΙΝΗΤΟ", end="\r")
        else:
            print(f"Status: ΚΙΝΗΣΗ -> Vx:{v[0]:.3f} Vy:{v[1]:.3f} Vz:{v[2]:.3f} Wz:{v[5]:.3f}", end="\r")
        time.sleep(1/30)