import pygame, time

pygame.init()
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()

print("--- ΔΙΑΓΝΩΣΤΙΚΟ ΑΞΟΝΩΝ ΧΕΙΡΙΣΤΗΡΙΟΥ ---")
print("Κουνήστε ΜΟΝΟ τον δεξιό μοχλό δεξιά/αριστερά και δείτε ποιος αριθμός αλλάζει:\n")

while True:
    pygame.event.pump()
    axes = [round(j.get_axis(i), 2) for i in range(j.get_numaxes())]
    print(f"Axes: {axes}  ", end="\r")
    time.sleep(0.1)
