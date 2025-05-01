import pygame
import carla
import logging
import time
import numpy as np
import cv2


# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()
control = carla.VehicleControl()

# Ensure that there is at least one joystick (G29 in this case)
if pygame.joystick.get_count() == 0:
    logging.error("No joystick detected.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

# Read inputs from the joystick (G29 wheel)
def get_steering_input():
    steering = joystick.get_axis(0) # Wheel rotation
    print(f"Steering Wheel: {steering: .2f}")
    return steering  # Axis 0 for steering (left-right)

def get_throttle_input():
    throttle = joystick.get_axis(1)
    print(f"Throttle: {throttle: .2f}")
    return min(throttle, 0.5)  # Axis 1 for throttle (forward/backward)

def get_brake_input():
    brake = joystick.get_axis(2)
    print(f"Brake: {brake: .2f}")
    return max(0.0, brake)  # Axis 2 for brake (left-right)

def get_gear_input(vehicle):
    control.manual_gear_shift = True
    control.gear = -1  # reverse
    current_control = vehicle.get_control()
    current_gear = current_control.gear
    print(f"Current gear: {current_gear}")

# Connect to CARLA
def connect_to_carla():
    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        logging.info("Successfully connected to CARLA.")
        return world
    except carla.ClientError as e:
        logging.error(f"Failed to connect to CARLA: {e}")
        return None

# Create vehicle control function
# Create vehicle control function
def control_vehicle(vehicle, steering, throttle, brake):
    control = carla.VehicleControl()
    control.steer = steering  # Steering value between -1 (full left) and 1 (full right)
    control.throttle = throttle  # Throttle value between 0 (no throttle) and 1 (full throttle)
    control.brake = brake  # Brake value between 0 (no brake) and 1 (full brake)
    
    # Ensure vehicle is in drive gear before moving
    if vehicle.get_control().gear != 1:  # Gear 1 is Drive mode
        control.gear = 1  # Set the gear to Drive mode
    
    vehicle.apply_control(control)

def camera_callback(image, data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

# Main function
def main():
    # Step 1: Connect to CARLA
    world = connect_to_carla()
    if not world:
        return

    # Step 2: Get a blueprint for the vehicle
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find('vehicle.tesla.model3')  # You can choose other vehicles as well
    spawn_points = world.get_map().get_spawn_points()

    # Step 3: Spawn the vehicle at a spawn point
    spawn_point = spawn_points[0]  # Just use the first spawn point
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    spectator = world.get_spectator()
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    CAMERA_POS_Z = 3
    CAMERA_POS_X = -5
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z, x = CAMERA_POS_X))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    sensor_data = {'image': np.zeros((image_h, image_w, 4))}
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RGB Camera', sensor_data['image'])
    cv2.waitKey(1)
    camera.listen(lambda image: camera_callback(image, sensor_data))
   

    # Step 4: Start the game loop and handle input
    try:
        while True:
            pygame.event.pump()  # Process any pygame events (important for joystick)

            # Step 5: Read G29 inputs
            steering = get_steering_input()
            throttle = get_throttle_input()
            brake = get_brake_input()
            get_gear_input(vehicle)
            # Step 6: Apply the controls to the vehicle
            control_vehicle(vehicle, steering, throttle, brake)
            # Update the display and check for the quit event
            cv2.imshow('RGB Camera', sensor_data['image'])
            if cv2.waitKey(1) == ord('q'):
                break
            # Step 7: Sleep for a short time to allow the vehicle to respond
            time.sleep(0.01)

    except KeyboardInterrupt:
        logging.info("Simulation ended.")
    finally:
        # Clean up by destroying the vehicle actor
        vehicle.destroy()

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
