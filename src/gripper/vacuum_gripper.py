import time

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except (ImportError, RuntimeError):
    RPI_AVAILABLE = False


class VacuumGripper:
    def __init__(self, arm_interface=None, pin_pump=20, pin_valve=21, use_gpio=False, mock=False):
        """
        Διαχείριση της βεντούζας αναρρόφησης (Vacuum Gripper).
        
        :param arm_interface: Το αντικείμενο MyArmInterface (αν χρησιμοποιείται το pymycobot I/O).
        :param pin_pump: Pin ενεργοποίησης της αντλίας αναρρόφησης (suction pump).
        :param pin_valve: Pin ενεργοποίησης της βαλβίδας εκτόνωσης (release valve), αν υπάρχει.
        :param use_gpio: Αν True, χρησιμοποιεί τα physical GPIO pins του Raspberry Pi.
        :param mock: Αν True, προσομοιώνει τη λειτουργία χωρίς πραγματικό υλικό.
        """
        self.mock = mock or (not RPI_AVAILABLE and use_gpio and arm_interface is None)
        self.use_gpio = use_gpio and RPI_AVAILABLE
        self.arm_interface = arm_interface
        self.pin_pump = pin_pump
        self.pin_valve = pin_valve
        self.is_active = False

        if self.mock:
            print("[GRIPPER] Αρχικοποίηση σε MOCK mode.")
        elif self.use_gpio:
            print(f"[GRIPPER] Χρήση Raspberry Pi GPIO (Pump: {self.pin_pump}, Valve: {self.pin_valve}).")
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.pin_pump, GPIO.OUT, initial=GPIO.LOW)
            if self.pin_valve is not None:
                GPIO.setup(self.pin_valve, GPIO.OUT, initial=GPIO.LOW)
        else:
            print(f"[GRIPPER] Χρήση MyArm I/O API (Pin: {self.pin_pump}).")

        self.release()

    def grab(self):
        """Ενεργοποιεί την αναρρόφηση για σύλληψη αντικειμένου."""
        if self.is_active:
            return

        self.is_active = True
        if self.mock:
            print("[GRIPPER - MOCK] Suction PUMP -> ON")
            return

        if self.use_gpio:
            if self.pin_valve is not None:
                GPIO.output(self.pin_valve, GPIO.LOW)  # Κλείσιμο βαλβίδας εκτόνωσης
            GPIO.output(self.pin_pump, GPIO.HIGH)       # Εκκίνηση αντλίας
        elif self.arm_interface and self.arm_interface.arm:
            try:
                # Στο pymycobot συνήθως 0 = ON για active-low relay, ή 1 = ON
                self.arm_interface.arm.set_basic_output(self.pin_pump, 0)
            except Exception as e:
                print(f"[GRIPPER ERROR] Grab failed: {e}")

    def release(self):
        """Απενεργοποιεί την αναρρόφηση και απελευθερώνει το αντικείμενο."""
        self.is_active = False
        if self.mock:
            print("[GRIPPER - MOCK] Suction PUMP -> OFF")
            return

        if self.use_gpio:
            GPIO.output(self.pin_pump, GPIO.LOW)        # Σβήσιμο αντλίας
            if self.pin_valve is not None:
                GPIO.output(self.pin_valve, GPIO.HIGH)  # Άνοιγμα βαλβίδας για άμεση εκτόνωση πίεσης
                time.sleep(0.15)
                GPIO.output(self.pin_valve, GPIO.LOW)
        elif self.arm_interface and self.arm_interface.arm:
            try:
                self.arm_interface.arm.set_basic_output(self.pin_pump, 1)
            except Exception as e:
                print(f"[GRIPPER ERROR] Release failed: {e}")

    def set_state(self, state: bool):
        """Ορίζει την κατάσταση: True = grab, False = release."""
        if state:
            self.grab()
        else:
            self.release()

    def cleanup(self):
        """Καθαρισμός των I/O κατά τον τερματισμό."""
        self.release()
        if self.use_gpio:
            GPIO.cleanup()