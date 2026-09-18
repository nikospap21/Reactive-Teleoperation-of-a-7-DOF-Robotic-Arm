import time
import numpy as np

try:
    from pymycobot import MyArm
    PYMYCOBOT_AVAILABLE = True
except ImportError:
    PYMYCOBOT_AVAILABLE = False


class MyArmInterface:
    def __init__(
        self,
        port="/dev/ttyAMA0",
        baudrate=115200,
        mock=False,
        joint_limits_lower=None,
        joint_limits_upper=None
    ):
        self.mock = mock
        self.port = port
        self.baudrate = baudrate
        self.num_joints = 7

        self.joint_limits_lower = joint_limits_lower if joint_limits_lower is not None else -np.pi * np.ones(7)
        self.joint_limits_upper = joint_limits_upper if joint_limits_upper is not None else np.pi * np.ones(7)

        # Εσωτερική κατάσταση για mock ή αρχικοποίηση
        self._mock_q = np.zeros(self.num_joints)

        self.arm = None
        if not self.mock:
            if not PYMYCOBOT_AVAILABLE:
                raise ImportError("[ERR] Η βιβλιοθήκη pymycobot δεν είναι εγκατεστημένη στο Raspberry Pi!")
            print(f"[*] Σύνδεση με MyArm στο {port} ({baudrate} baud)...")
            self.arm = MyArm(self.port, self.baudrate)
            time.sleep(0.5)
            self.arm.power_on()
            time.sleep(0.5)
            print("[+] MyArm συνδέθηκε επιτυχώς!")

    def get_joint_angles(self):
        """Επιστρέφει τις γωνίες των 7 αρθρώσεων σε radians."""
        if self.mock:
            return np.copy(self._mock_q)

        try:
            angles_deg = self.arm.get_radians()
            if angles_deg and len(angles_deg) == 7:
                return np.array(angles_deg, dtype=float)
            else:
                # Εναλλακτική ανάγνωση αν επιστρέφει μοίρες
                deg = self.arm.get_angles()
                if deg and len(deg) == 7:
                    return np.deg2rad(np.array(deg, dtype=float))
        except Exception as e:
            print(f"[WARN] Σφάλμα ανάγνωσης αρθρώσεων: {e}")

        return np.copy(self._mock_q)

    def send_joint_velocities(self, q_dot, q_current, dt, speed=80):
        """
        Υπολογίζει q_target = q_current + q_dot * dt και στέλνει εντολή θέσης/ταχύτητας.
        """
        q_target = q_current + q_dot * dt
        # Περιορισμός εντός ορίων ασφαλείας
        q_target = np.clip(q_target, self.joint_limits_lower, self.joint_limits_upper)

        if self.mock:
            self._mock_q = np.copy(q_target)
            return

        try:
            target_deg = np.rad2deg(q_target).tolist()
            # Αποστολή γωνιών στο MyArm
            self.arm.send_angles(target_deg, speed)
        except Exception as e:
            print(f"[ERR] Αποτυχία αποστολής εντολής στο MyArm: {e}")

    def go_home(self, speed=30):
        """Μετάβαση στην αρχική θέση (όλες οι αρθρώσεις στο μηδέν)."""
        home = [0.0] * 7
        if self.mock:
            self._mock_q = np.array(home)
            return

        try:
            self.arm.send_angles(home, speed)
            time.sleep(1.5)
        except Exception as e:
            print(f"[WARN] Σφάλμα go_home: {e}")

    def stop_motion(self):
        """Ακινητοποίηση του βραχίονα."""
        if not self.mock and self.arm is not None:
            try:
                # Διατήρηση της τρέχουσας θέσης
                q = self.get_joint_angles()
                self.arm.send_angles(np.rad2deg(q).tolist(), 10)
            except Exception:
                pass
