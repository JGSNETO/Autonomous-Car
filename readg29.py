import pygame
import time
import logging

# Initialize pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
 
# Check how many joystick(controllers) are connected
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    logging.info("No joystick connected!")

else:
    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        logging.info(f"Connected to: {joystick.get_name()}")

    except Exception as e:
        logging.error("Error while trying to connect with joystick: {e}")

try:
    while True:
        pygame.event.pump() # Process the events

        # Get the wheel axis value(Steering Wheel)
        steering = joystick.get_axis(0) # Wheel rotation
        print(f"Steering Wheel: {steering: .2f}")

        # Get the pedals 
        throttle = joystick.get_axis(1)
        brake = joystick.get_axis(2)
        clutch = joystick.get_axis(3)

        print(f"Throttle: {throttle: .2f}, Brake: {brake: .2f}, Clutch: {clutch: 2f}")

        time.sleep(0.1)
except Exception as e:
    logging.error(f"Error while acquiring the control input: {e}")

finally:
    pygame.quit()
