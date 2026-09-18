import time
from pymycobot import MyArm #type:ignore

# Δοκιμή με την εσωτερική θύρα ttyAMA0 (ή ttyUSB0 αν χρησιμοποιείτε προσαρμογέα)
PORT = "/dev/ttyAMA0"
BAUD = 115200
print(f"Προσπάθεια σύνδεσης στη θύρα {PORT}...")
arm = MyArm(PORT, BAUD)

# Ενεργοποίηση Servos
arm.power_on()
time.sleep(1)

# Ανάγνωση τρεχουσών γωνιών
angles = arm.get_angles()
print(f"Τρέχουσες γωνίες αρθρώσεων (degrees): {angles}")

# Μικρή δοκιμαστική κίνηση στην 1η άρθρωση (Base) κατά +10 μοίρες
if angles and len(angles) == 7:
    target_angles = list(angles)
    target_angles[0] += 10.0  # Στρίβει ελαφρώς τη βάση
    print(f"Αποστολή νέων γωνιών: {target_angles}")
    arm.send_angles(target_angles, 20)  # Αργή ταχύτητα 20%
    time.sleep(2)
    
    # Επαναφορά
    arm.send_angles(angles, 20)
    print("Δοκιμή ολοκληρώθηκε!")
else:
    print("Σφάλμα: Δεν λήφθηκαν έγκυρες γωνίες από το ρομπότ.")