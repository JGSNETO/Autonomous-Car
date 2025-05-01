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

def get_steering_input():
    steering = joystick.get_axis(0) # Wheel rotation
    print(f"Steering Wheel: {steering: .2f}")
    return steering  # Axis 0 for steering (left-right)

def get_throttle_input():
    throttle = joystick.get_axis(1)
    print(f"Throttle: {max(0.0, -throttle*0.5): .2f}")
    return max(0.0, -throttle*0.5)  # Axis 1 for throttle (forward/backward)

def get_brake_input():
    brake = joystick.get_axis(2)
    print(f"Brake: {max(0.0, -brake): .2f}")
    return max(0.0, -brake)  # Axis 2 for brake (left-right)

try:
    while True:
        pygame.event.pump() # Process the events

        # Get the wheel axis value(Steering Wheel)
        steering = get_steering_input() # Wheel rotation
        print(f"Steering Wheel: {steering: .2f}")

        # Get the pedals 
        throttle = get_throttle_input()
        brake = get_brake_input()
        clutch = joystick.get_axis(3)

        print(f"Throttle: {throttle: .2f}, Brake: {brake: .2f}, Clutch: {clutch: 2f}")

        time.sleep(0.1)

except Exception as e:
    logging.error(f"Error while acquiring the control input: {e}")

finally:
    pygame.quit()
