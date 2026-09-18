import os
import yaml
import numpy as np
import pygame

# Απενεργοποίηση video/display requirements του pygame για headless/terminal χρήση
os.environ["SDL_VIDEODRIVER"] = "dummy"

# 1. Βρίσκουμε τον φάκελο στον οποίο βρίσκεται το τρέχον script (π.χ. src/input/)
CURRENT_FILE_DIR = os.path.dirname(os.path.abspath(__file__))

# 2. Ανεβαίνουμε δύο επίπεδα (.. -> src, .. -> project root) και πάμε στο config/
CONFIG_PATH = os.path.abspath(os.path.join(CURRENT_FILE_DIR, "..", "..", "config", "controller_config.yaml"))

# 3. Ασφαλές άνοιγμα ανεξάρτητα από πού εκτελείται η εντολή στο terminal
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    config_data = yaml.safe_load(f)


class DirectPS2Controller:
    def __init__(self, config_path="config/controller_config.yaml"):
        # Φόρτωση ρυθμίσεων
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = {}

        self.deadzone = self.config.get("deadzone", 0.10)
        self.scale_linear = self.config.get("scale_linear", 0.05)   # m/s
        self.scale_angular = self.config.get("scale_angular", 0.20) # rad/s

        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            print("[WARN] Δεν εντοπίστηκε joystick στο USB του Raspberry Pi! Ελέγξτε τη σύνδεση.")
            self.joystick = None
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[+] Εντοπίστηκε Controller: {self.joystick.get_name()}")

        self.gripper_state = False
        self.prev_btn_state = False

    def _apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self.deadzone) / (1.0 - self.deadzone)

    def update(self):
        """
        Επιστρέφει:
            vx, vy, vz (m/s)
            wz (rad/s - περιστροφή yaw)
            gripper_state (bool)
            is_connected (bool)
        """
        if self.joystick is None:
            return 0.0, 0.0, 0.0, 0.0, False, False

        pygame.event.pump()

        # Άξονες (Τυπικό PS2 / Gamepad mapping)
        # Left Stick: X (Axis 0), Y (Axis 1 - αντεστραμμένο στο pygame)
        # Right Stick: Z/Elevation (Axis 3/4), Yaw (Axis 2)
        raw_lx = self.joystick.get_axis(0)
        raw_ly = -self.joystick.get_axis(1)
        
        # Ανάλογα με το mapping του controller
        raw_rx = self.joystick.get_axis(2) if self.joystick.get_numaxes() > 2 else 0.0
        raw_ry = -self.joystick.get_axis(3) if self.joystick.get_numaxes() > 3 else 0.0

        vx = self._apply_deadzone(raw_ly) * self.scale_linear
        vy = -self._apply_deadzone(raw_lx) * self.scale_linear
        vz = self._apply_deadzone(raw_ry) * self.scale_linear
        wz = self._apply_deadzone(raw_rx) * self.scale_angular

        # Κουμπιά για Vacuum Gripper (π.χ. Button 0 = X/Cross, Button 1 = Circle/B)
        # Ή toggle με R1 (Button 5)
        num_buttons = self.joystick.get_numbuttons()
        if num_buttons > 0:
            btn_on = self.joystick.get_button(0)   # Ενεργοποίηση
            btn_off = self.joystick.get_button(1) if num_buttons > 1 else False

            if btn_on:
                self.gripper_state = True
            elif btn_off:
                self.gripper_state = False

        return vx, vy, vz, wz, self.gripper_state, True
