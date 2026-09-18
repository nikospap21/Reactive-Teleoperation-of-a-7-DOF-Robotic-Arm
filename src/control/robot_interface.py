import numpy as np
from pymycobot.myarm import MyArm


class MyArmInterface:

  def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
    # Στο Raspberry Pi του MyArm η σειριακή επικοινωνία είναι συνήθως /dev/ttyAMA0
    try:
      self.arm = MyArm(port, baudrate)
      self.arm.power_on()
      print(f"[ROBOT] Επιτυχής σύνδεση στη θύρα {port}")
    except Exception as e:
      print(f"[ROBOT ERR] Αποτυχία σύνδεσης στο {port}: {e}")
      self.arm = None

  def send_joint_angles(self, angles_rad, speed=40):
    """Στέλνει γωνίες (σε μοίρες) στους 7 κινητήρες."""
    if self.arm is not None:
      angles_deg = [float(np.degrees(a)) for a in angles_rad]
      self.arm.send_angles(angles_deg, speed)

  def get_joint_angles(self):
    """Διαβάζει τις πραγματικές γωνίες σε ακτίνια."""
    if self.arm is not None:
      deg = self.arm.get_angles()
      if deg and len(deg) == 7:
        return [np.radians(a) for a in deg]
    return None

  def stop(self):
    if self.arm is not None:
      self.arm.stop()