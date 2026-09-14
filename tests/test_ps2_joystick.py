import sys
import time
import pygame

def main():
    # Αρχικοποίηση του υποσυστήματος joystick του pygame
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("\nΔεν εντοπίστηκε χειριστήριο. Σύνδεσε το PS2 controller μέσω USB και ξαναδοκίμασε.\n")
        sys.exit(1)

    controller = pygame.joystick.Joystick(0)
    controller.init()

    print(f"\nΣυνδέθηκε: {controller.get_name()}")
    print(f"Άξονες: {controller.get_numaxes()} | Κουμπιά: {controller.get_numbuttons()} | Hats (D-pad): {controller.get_numhats()}")
    print("-" * 65)
    print("Κούνα τους μοχλούς και πάτα κουμπιά. Πάτα Ctrl+C για έξοδο.")
    print("-" * 65 + "\n")

    try:
        while True:
            # Επεξεργασία events για να ανανεωθούν οι τιμές
            pygame.event.pump()

            # Ανάγνωση αξόνων (στρογγυλοποίηση για ευανάγνωστη έξοδο)
            axes_vals = [round(controller.get_axis(i), 2) for i in range(controller.get_numaxes())]
            
            # Εντοπισμός πατημένων κουμπιών
            buttons_pressed = [i for i in range(controller.get_numbuttons()) if controller.get_button(i)]

            # Καθαρή εκτύπωση σε μία γραμμή (\r)
            axes_str = " | ".join([f"A{i}: {val:>5}" for i, val in enumerate(axes_vals)])
            btn_str = f"Buttons: {buttons_pressed if buttons_pressed else 'None'}"
            
            print(f"\r{axes_str}  ||  {btn_str:<25}", end="", flush=True)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nΤερματισμός ελέγχου.")
        pygame.quit()

if __name__ == "__main__":
    main()
