import time


class VacuumGripper:
    def __init__(self, arm_interface, pin_pump=20, pin_valve=21, mock=False):
        self.arm_interface = arm_interface
        self.pin_pump = pin_pump
        self.pin_valve = pin_valve
        self.mock = mock
        self.is_active = False

    def set_state(self, active: bool):
        self.is_active = active
        if self.mock:
            return

        try:
            arm = self.arm_interface.arm
            if arm is not None:
                if active:
                    # Ενεργοποίηση αντλίας αναρρόφησης (Basic output)
                    arm.set_basic_output(self.pin_pump, 0)
                    arm.set_basic_output(self.pin_valve, 1)
                else:
                    # Απενεργοποίηση αντλίας και άνοιγμα βαλβίδας απελευθέρωσης
                    arm.set_basic_output(self.pin_pump, 1)
                    arm.set_basic_output(self.pin_valve, 0)
                    time.sleep(0.1)
                    arm.set_basic_output(self.pin_valve, 1)
        except Exception as e:
            print(f"[WARN] Σφάλμα ελέγχου Suction Gripper: {e}")

    def cleanup(self):
        self.set_state(False)
