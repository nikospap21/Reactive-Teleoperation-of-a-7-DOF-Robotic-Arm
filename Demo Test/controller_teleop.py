import pygame #type:ignore
import numpy as np

class PS2Controller:
    def __init__(self, device_index=0, deadzone=0.20):
        """
        Αρχικοποίηση PS2 Controller με αυστηρό φιλτράρισμα drift.
        """
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Σφάλμα: Δεν βρέθηκε χειριστήριο συνδεδεμένο!")

        self.joystick = pygame.joystick.Joystick(device_index)
        self.joystick.init()
        
        self.deadzone = deadzone
        self.vacuum_state = False

        # Scalers Ταχυτήτων (m/s & rad/s)
        self.scale_pos = 0.04
        self.scale_rot = 0.15

    def _apply_deadzone(self, value):
        """Αυστηρή νεκρή ζώνη: Μηδενίζει οτιδήποτε μικρότερο από το deadzone."""
        if abs(value) < self.deadzone:
            return 0.0
        # Smooth scaling μετά τη νεκρή ζώνη
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def get_cartesian_velocity(self):
        """
        Διαβάζει ΑΠΟΚΛΕΙΣΤΙΚΑ τους 2 αναλογικούς μοχλούς.
        - Left Stick (Axes 0, 1) -> X / Y κίνηση
        - Right Stick (Axes 2, 3) -> Z κίνηση & Yaw περιστροφή
        """
        pygame.event.pump()

        # Ανάγνωση Αξόνων (με ασφάλεια για υπαρκτούς άξονες μόνο)
        num_axes = self.joystick.get_numaxes()
        
        raw_lx = self.joystick.get_axis(0) if num_axes > 0 else 0.0
        raw_ly = self.joystick.get_axis(1) if num_axes > 1 else 0.0
        raw_rx = self.joystick.get_axis(2) if num_axes > 2 else 0.0
        raw_ry = self.joystick.get_axis(3) if num_axes > 3 else 0.0

        # Εφαρμογή νεκρής ζώνης
        lx = self._apply_deadzone(raw_lx)
        ly = self._apply_deadzone(raw_ly)
        rx = self._apply_deadzone(raw_rx)
        ry = self._apply_deadzone(raw_ry)

        # Αντιστοίχιση σε Καρτεσιανές Ταχύτητες [vx, vy, vz, wx, wy, wz]
        vx = -ly * self.scale_pos  # Πρόσω / Πίσω
        vy =  lx * self.scale_pos  # Αριστερά / Δεξιά
        vz = -ry * self.scale_pos  # Πάνω / Κάτω
        
        wx = 0.0
        wy = 0.0
        wz = 0.0 #  rx * self.scale_rot  # Περιστροφή End-Effector

        v_cmd = np.array([vx, vy, vz, wx, wy, wz])

        # Εγγυημένος μηδενισμός αν η συνολική εντολή είναι ελάχιστη
        if np.max(np.abs(v_cmd)) < 0.001:
            return np.zeros(6)

        return v_cmd

    def process_gripper_events(self):
        """A (Grasp) & B (Release) Buttons."""
        pygame.event.pump()

        if self.joystick.get_numbuttons() > 0 and self.joystick.get_button(0):
            if not self.vacuum_state:
                self.vacuum_state = True
                print("\n >>> [ACTION: GRASP (Button A)] Suction Pump: ON <<< \n")

        elif self.joystick.get_numbuttons() > 1 and self.joystick.get_button(1):
            if self.vacuum_state:
                self.vacuum_state = False
                print("\n >>> [ACTION: RELEASE (Button B)] Suction Pump: OFF <<< \n")

        return self.vacuum_state

# --- TEST STANDALONE ---
if __name__ == "__main__":
    import time
    ctrl = PS2Controller()
    print("--- ΤΕΣΤ ΧΕΙΡΙΣΤΗΡΙΟΥ (Αφήστε τους μοχλούς) ---")
    while True:
        v = ctrl.get_cartesian_velocity()
        if np.linalg.norm(v) == 0:
            print("Status: ΑΚΙΝΗΤΟ (Velocity = 0.000)                ", end="\r")
        else:
            print(f"Status: ΚΙΝΗΣΗ -> Vx:{v[0]:.3f} Vy:{v[1]:.3f} Vz:{v[2]:.3f} Wz:{v[5]:.3f}", end="\r")
        time.sleep(1/30)
