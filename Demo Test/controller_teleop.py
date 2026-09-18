import pygame #  type:ignore
import numpy as np

class PS2Controller:
    def __init__(self, device_index=0, deadzone=0.08):
        """
        Αρχικοποίηση PS2 Controller & Παραμέτρων Mapping
        """
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Σφάλμα: Δεν βρέθηκε χειριστήριο PS2 συνδεδεμένο!")

        self.joystick = pygame.joystick.Joystick(device_index)
        self.joystick.init()
        print(f"[PS2 Controller] Ενεργό: {self.joystick.get_name()}")

        self.deadzone = deadzone
        self.vacuum_state = False  # False = Released, True = Grasped

        # Scales Ταχυτήτων (m/s για μετατόπιση, rad/s για περιστροφή)
        self.scale_pos = 0.05
        self.scale_rot = 0.2

    def _apply_deadzone(self, value):
        """Φιλτράρισμα νεκρής ζώνης για αποφυγή drift."""
        return 0.0 if abs(value) < self.deadzone else value

    def get_cartesian_velocity(self):
        """
        Διαβάζει τους μοχλούς και επιστρέφει το διάνυσμα ταχύτητας v (6x1):
        v = [vx, vy, vz, wx, wy, wz]
        """
        pygame.event.pump()

        # Ανάγνωση Αξόνων
        lx = self._apply_deadzone(self.joystick.get_axis(0))  # Left/Right
        ly = self._apply_deadzone(self.joystick.get_axis(1))  # Forward/Backward
        rx = self._apply_deadzone(self.joystick.get_axis(2))  # Yaw Rotation
        ry = self._apply_deadzone(self.joystick.get_axis(3))  # Up/Down

        # Αντιστοίχιση σε Καρτεσιανές Ταχύτητες
        vx = -ly * self.scale_pos  # Πρόσω / Πίσω
        vy =  lx * self.scale_pos  # Αριστερά / Δεξιά
        vz = -ry * self.scale_pos  # Πάνω / Κάτω
        
        wx = 0.0                   # Roll (σταθερό)
        wy = 0.0                   # Pitch (σταθερό)
        wz =  rx * self.scale_rot  # Yaw (περιστροφή end-effector)

        return np.array([vx, vy, vz, wx, wy, wz])

    def process_gripper_events(self):
        """
        Διαχειρίζεται τα κουμπιά L1 (Grasp) και R1 (Release)
        και εμφανίζει ξεκάθαρο μήνυμα στην κονσόλα.
        """
        pygame.event.pump()

        # Button 4 = L1 (Grasp / Suction ON)
        if self.joystick.get_button(0):
            if not self.vacuum_state:
                self.vacuum_state = True
                print("\n" + "="*40)
                print(" >>> [ACTION: GRASP] Suction Pump: ON <<<")
                print("="*40 + "\n")

        # Button 5 = R1 (Release / Suction OFF)
        elif self.joystick.get_button(1):
            if self.vacuum_state:
                self.vacuum_state = False
                print("\n" + "="*40)
                print(" >>> [ACTION: RELEASE] Suction Pump: OFF <<<")
                print("="*40 + "\n")

        return self.vacuum_state

# --- Αυτόνομη Δοκιμή (Standalone Test) ---
if __name__ == "__main__":
    import time
    try:
        controller = PS2Controller()
        print("\n--- TEST MODE: Κουνήστε τους μοχλούς & πατήστε L1 / R1 ---")
        
        while True:
            v_cmd = controller.get_cartesian_velocity()
            gripper_status = controller.process_gripper_events()

            # Τύπωμα ταχυτήτων αν υπάρχει κίνηση
            if np.linalg.norm(v_cmd) > 0:
                print(f"Velocities -> Vx: {v_cmd[0]:.3f} | Vy: {v_cmd[1]:.3f} | Vz: {v_cmd[2]:.3f} | Wz: {v_cmd[5]:.3f}", end="\r")

            time.sleep(1/30)  # 30 Hz Loop

    except KeyboardInterrupt:
        print("\nΤερματισμός δοκιμής.")
