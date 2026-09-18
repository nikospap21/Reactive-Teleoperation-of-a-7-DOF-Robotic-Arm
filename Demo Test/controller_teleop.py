import pygame #type:ignore
import numpy as np

class PS2Controller:
    def __init__(self, device_index=0, deadzone = 0.1):
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
        self.scale_pos = 0.1
        self.scale_pos_z = 0.2
        self.scale_rot = 0.3

    def _apply_deadzone(self, value):
        """Αυστηρή νεκρή ζώνη: Μηδενίζει οτιδήποτε μικρότερο από το deadzone."""
        if abs(value) < self.deadzone:
            return 0.0
        # Smooth scaling μετά τη νεκρή ζώνη
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def normalize_trigger(self, raw_value):
        norm_val = (raw_value + 1.0) / 2.0
        return self._apply_deadzone(norm_val)

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
        raw_lz = self.joystick.get_axis(4) if num_axes > 4 else 0.0
        raw_rx = self.joystick.get_axis(2) if num_axes > 2 else -1.0
        raw_ry = self.joystick.get_axis(5) if num_axes > 5 else -1.0
        raw_rz = self.joystick.get_axis(3) if num_axes > 3 else 0.0

        # Εφαρμογή νεκρής ζώνης
        lx = self._apply_deadzone(raw_lx)
        ly = self._apply_deadzone(raw_ly)
        lz = self._apply_deadzone(raw_lz)
        rx = self.normalize_trigger(raw_rx)
        ry = self.normalize_trigger(raw_ry)
        rz = self._apply_deadzone(raw_rz)


        # Αντιστοίχιση σε Καρτεσιανές Ταχύτητες [vx, vy, vz, wx, wy, wz]
        vx = -lx * self.scale_pos  # Πρόσω / Πίσω
        vy =  ly * self.scale_pos  # Αριστερά / Δεξιά
        vz = lz * self.scale_pos_z  # Πάνω / Κάτω
        
        wx = rx * self.scale_rot  # Yaw Περιστροφή
        wy = ry * self.scale_rot  # Yaw Περιστροφή          
        wz = rz * self.scale_rot  # Yaw Περιστροφή

        # Noise Cutoffs
        #if abs(vx) < 0.002: vx = 0.0
        #if abs(vy) < 0.002: vy = 0.0
        #if abs(vz) < 0.002: vz = 0.0
        #if abs(wz) < 0.020: wz = 0.0

        return np.array([vx, vy, vz, wx, wy, wz])

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

    def process_reset_event(self):
        """
        Ελέγχει αν πατήθηκε το κουμπί RESET (π.χ. Start Button).
        Επιστρέφει True αν πατήθηκε, αλλιώς False.
        """
        pygame.event.pump()
        num_buttons = self.joystick.get_numbuttons()

        # Κουμπί Start (συνήθως index 7 ή 9) ή κουμπί X/Cross (index 2)
        reset_button_indices = [7, 9, 2]

        for btn_idx in reset_button_indices:
            if num_buttons > btn_idx and self.joystick.get_button(btn_idx):
                print("\n >>> [ACTION: HOME RESET] Επαναφορά στη μηδενική θέση... <<< \n")
                return True

        return False

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
