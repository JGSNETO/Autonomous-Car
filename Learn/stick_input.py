import pygame
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected joystick: {joystick.get_name()}")

try:
    while True:
        pygame.event.pump()

        # Read button presses for paddle shifters
        shift_up = joystick.get_button(5)  # Right paddle (upshift)
        shift_down = joystick.get_button(4)  # Left paddle (downshift)

        if shift_up:
            print("Up-shift (Right Paddle) pressed")
        if shift_down:
            print("Down-shift (Left Paddle) pressed")

        # Optionally, print the current state of other buttons
        for i in range(joystick.get_numbuttons()):
            print(f"Button {i}: {joystick.get_button(i)}", end=' | ')

        print("\n" + "-" * 50)  # Separator

        time.sleep(0.2)  # Adjust speed of polling

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
