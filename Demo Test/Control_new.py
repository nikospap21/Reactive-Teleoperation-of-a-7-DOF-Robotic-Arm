import time
import pygame
from pymycobot.myarm300 import MyArm300

# Initialize joystick framework
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ Error: No Xbox controller found!")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"🎮 Connected to: {controller.get_name()}")

# Initialize myArm 300 Pi 
# Note: Use "/dev/ttyAMA0" if running directly on the Pi. Change to your COM port if running from a PC.
arm = MyArm300("/dev/ttyAMA0", 115200)
arm.power_on()
time.sleep(1)

print("\n🚀 Cartesian Teleoperation Started!")
print("---------------------------------------")
print("• Left Stick         -> Move X / Y")
print("• Triggers (LT/RT)   -> Move Z (Down / Up)")
print("• Right Stick        -> Rotate Rx / Ry")
print("• Bumpers (LB/RB)    -> Rotate Rz (Clockwise / Counter)")
print("• Press 'Back' Button -> Exit & Power Down")
print("---------------------------------------")

speed = 30       # Movement speed (1-100)
deadzone = 0.15  # Prevents stick drift stick movement
running = True

while running:
    pygame.event.pump()
    
    # 1. Read Inputs & Handle Deadzones
    ls_x = controller.get_axis(0) if abs(controller.get_axis(0)) > deadzone else 0
    ls_y = -controller.get_axis(1) if abs(controller.get_axis(1)) > deadzone else 0 # Invert Y
    
    rs_x = controller.get_axis(2) if abs(controller.get_axis(2)) > deadzone else 0
    rs_y = -controller.get_axis(3) if abs(controller.get_axis(3)) > deadzone else 0 # Invert Y

    # Triggers usually range from -1.0 (released) to 1.0 (fully pressed)
    lt = (controller.get_axis(4) + 1) / 2  # Normalize to 0.0 - 1.0
    rt = (controller.get_axis(5) + 1) / 2  # Normalize to 0.0 - 1.0
    
    # Bumpers for Rz rotation
    lb = controller.get_button(4)
    rb = controller.get_button(5)
    
    # Exit condition (Back/View button)
    if controller.get_button(6):
        running = False
        break

    # 2. Calculate Deltas (Scales the movement magnitude)
    dx = ls_x * 8.0   # Move X
    dy = ls_y * 8.0   # Move Y
    dz = (rt - lt) * 8.0  # Move Z (Right Trigger updates up, Left Trigger updates down)
    
    drx = rs_x * 4.0  # Pitch / Roll rotations
    dry = rs_y * 4.0
    drz = (rb - lb) * 4.0 # Yaw rotation

    # 3. Update Coordinates if movement is requested
    if any([dx, dy, dz, drx, dry, drz]):
        current_coords = arm.get_coords()
        
        if current_coords and len(current_coords) == 6:
            # Apply changes to current position [x, y, z, rx, ry, rz]
            current_coords[0] += dx
            current_coords[1] += dy
            current_coords[2] += dz
            current_coords[3] += drx
            current_coords[4] += dry
            current_coords[5] += drz
            
            # Send updated Cartesian targets (Mode 1 = Linear Path interpolation)
            arm.send_coords(current_coords, speed, 1)
            
    time.sleep(0.04)  # 25Hz refresh rate prevents serial port choking

# Safe Shutdown
arm.power_off()
print("🛑 Teleoperation stopped safely.")
