import json
import socket
import yaml


class PS2Receiver:

  def __init__(self, config_path="config/controller_config.yaml"):
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