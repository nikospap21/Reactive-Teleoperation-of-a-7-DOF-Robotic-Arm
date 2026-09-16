import time
import numpy as np

try:
  from pymycobot.myarm import MyArm
except ImportError:
  MyArm = None


class MyArmInterface:

  def __init__(
      self,
      port="/dev/ttyAMA0",
      baudrate=115200,
      mock=False,
      joint_limits_lower=None,
      joint_limits_upper=None,
  ):
    """Interface για τον Elephant Robotics MyArm 300 Pi.

    port: '/dev/ttyAMA0' (στο Pi) ή '/dev/ttyUSB0' (μέσω USB)
    mock: Αν είναι True, τρέχει σε simulation/offline mode χωρίς πραγματικό ρομπότ.
    """
    self.mock = mock or (MyArm is None)
    self.port = port
    self.baudrate = baudrate

    # Όρια ασφαλείας (σε μοίρες)
    self.q_min_deg = (
        np.degrees(joint_limits_lower)
        if joint_limits_lower is not None
        else np.array([-165.0] * 6 + [-175.0])
    )
    self.q_max_deg = (
        np.degrees(joint_limits_upper)
        if joint_limits_upper is not None
        else np.array([165.0] * 6 + [175.0])
    )

    self.arm = None
    if not self.mock:
      try:
        print(f"[*] Σύνδεση με MyArm 300 Pi στη θύρα {port} ({baudrate} bps)...")
        self.arm = MyArm(port, baudrate)
        time.sleep(0.5)
        # Ενεργοποίηση των σερβοκινητήρων
        self.arm.power_on()
        time.sleep(0.2)
        print("[+] Επιτυχής σύνδεση με το πραγματικό ρομπότ.")
      except Exception as e:
        print(
            f"[-] Αποτυχία σύνδεσης στο {port}: {e}. Ενεργοποίηση Mock Mode."
        )
        self.mock = True
    else:
      print("[!] Εκτέλεση σε MOCK MODE (Χωρίς πραγματικό υλικό).")

    # Εσωτερική κατάσταση για mock mode (σε μοίρες)
    self._mock_angles = [0.0, 30.0, 0.0, 60.0, 0.0, 30.0, 0.0]

  def get_joint_angles(self):
    """Επιστρέφει τις τρέχουσες γωνίες των 7 αρθρώσεων σε radians."""
    if self.mock:
      return np.radians(self._mock_angles)

    try:
      angles_deg = self.arm.get_angles()
      # Έλεγχος εγκυρότητας λίστας από το pymycobot
      if (
          angles_deg
          and isinstance(angles_deg, list)
          and len(angles_deg) == 7
          and not any(a is None for a in angles_deg)
      ):
        return np.radians(angles_deg)
    except Exception as e:
      print(f"[WARN] Αποτυχία ανάγνωσης γωνιών: {e}")

    # Fallback στην τελευταία γνωστή κατάσταση
    return np.radians(self._mock_angles)

  def send_joint_velocities(self, q_dot, q_current, dt, speed=90):
    """Μετατρέπει τις ταχύτητες q_dot (rad/s) σε στόχο γωνιών q_next (deg)

    και τις αποστέλλει στο ρομπότ.
    """
    # 1. Υπολογισμός επόμενης θέσης: q_next = q + q_dot * dt
    q_next_rad = q_current + q_dot * dt

    # 2. Μετατροπή σε μοίρες
    q_next_deg = np.degrees(q_next_rad)

    # 3. Αυστηρό hardware limit clamping
    q_next_deg = np.clip(q_next_deg, self.q_min_deg, self.q_max_deg)
    target_list = [round(float(a), 2) for a in q_next_deg]

    if self.mock:
      self._mock_angles = target_list
      return

    try:
      # Αποστολή των 7 γωνιών με καθορισμένη ταχύτητα (1-100)
      self.arm.send_angles(target_list, speed)
      self._mock_angles = target_list
    except Exception as e:
      print(f"[WARN] Σφάλμα αποστολής γωνιών: {e}")

  def stop_motion(self):
    """Ακινητοποιεί άμεσα τον βραχίονα."""
    if not self.mock and self.arm:
      try:
        self.arm.stop()
      except Exception as e:
        print(f"[WARN] Σφάλμα στο stop: {e}")

  def go_home(self, speed=40):
    """Επαναφέρει τον βραχίονα σε ασφαλή αρχική στάση."""
    home_deg = [0.0, 30.0, 0.0, 60.0, 0.0, 30.0, 0.0]
    print("[*] Μετάβαση στην αρχική θέση (Home Pose)...")
    if not self.mock and self.arm:
      self.arm.send_angles(home_deg, speed)
      time.sleep(2.0)
    self._mock_angles = home_deg