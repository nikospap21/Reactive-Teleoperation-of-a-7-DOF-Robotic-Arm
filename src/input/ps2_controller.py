import json
import socket
import yaml


class PS2Receiver:

  def __init__(self, config_path="config/controimport os
import yaml
import numpy as np

# Απενεργοποίηση video/display requirements του pygame για headless/terminal χρήση
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame


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

        return vx, vy, vz, wz, self.gripper_state, Trueller_config.yaml"):
    with open(config_path, "r", encoding="utf-8") as f:
      self.cfg = yaml.safe_load(f)

    # Ανάγνωση παραμέτρων δικτύου
    self.ip = self.cfg["network"]["ip"]
    self.port = self.cfg["network"]["port"]

    # Ανάγνωση controller παραμέτρων
    self.deadzone = float(self.cfg["controller"]["deadzone"])

    # Ανάγνωση axes mapping
    self.axis_x_idx = self.cfg["axes"]["axis_x"]
    self.axis_y_idx = self.cfg["axes"]["axis_y"]
    self.axis_z_idx = self.cfg["axes"]["axis_z"]
    self.rot_idx = self.cfg["axes"]["rot"]

    # Ανάγνωση scales
    self.scale_x = float(self.cfg["scales"]["linear_x"])
    self.scale_y = float(self.cfg["scales"]["linear_y"])
    self.scale_z = float(self.cfg["scales"]["linear_z"])
    self.scale_yaw = float(self.cfg["scales"]["angular_yaw"])

    # Ανάγνωση buttons
    self.btn_on_idx = self.cfg["buttons"]["gripper_on"]
    self.btn_off_idx = self.cfg["buttons"]["gripper_off"]

    # Κατάσταση gripper (False = OFF, True = ON)
    self.gripper_state = False

    # UDP Socket Setup
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind((self.ip, self.port))
    self.sock.setblocking(False)

    self.last_data = {"axes": [], "buttons": []}

  def _apply_deadzone(self, value):
    if abs(value) < self.deadzone:
      return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

  def update(self):
    """Αντλεί το πιο πρόσφατο UDP πακέτο και αδειάζει την ουρά."""
    received_any = False
    while True:
      try:
        data, _ = self.sock.recvfrom(1024)
        self.last_data = json.loads(data.decode("utf-8"))
        received_any = True
      except (BlockingIOError, socket.error):
        break
      except Exception:
        break

    axes = self.last_data.get("axes", [])
    buttons = self.last_data.get("buttons", [])

    def get_val(container, idx, default=0.0):
      if isinstance(container, list):
        return container[idx] if idx < len(container) else default
      if isinstance(container, dict):
        return container.get(str(idx), container.get(idx, default))
      return default

    # Ανάγνωση αξόνων
    raw_x = get_val(axes, self.axis_x_idx, 0.0)
    raw_y = get_val(axes, self.axis_y_idx, 0.0)
    raw_z = get_val(axes, self.axis_z_idx, 0.0)
    raw_rot = get_val(axes, self.rot_idx, 0.0)

    # Υπολογισμός καρτεσιανών ταχυτήτων
    vx = self._apply_deadzone(raw_x) * self.scale_x
    vy = self._apply_deadzone(raw_y) * self.scale_y
    vz = self._apply_deadzone(raw_z) * self.scale_z
    wz = self._apply_deadzone(raw_rot) * self.scale_yaw

    # Ενημέρωση κατάστασης gripper
    btn_on = get_val(buttons, self.btn_on_idx, 0)
    btn_off = get_val(buttons, self.btn_off_idx, 0)

    if btn_on == 1:
      self.gripper_state = True
    elif btn_off == 1:
      self.gripper_state = False

    return vx, vy, vz, wz, self.gripper_state, received_any
