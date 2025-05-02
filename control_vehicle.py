import pygame
import carla
import logging
import time
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm
import math
import matplotlib

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
    #print(f"Steering Wheel: {steering: .2f}")
    return steering  # Axis 0 for steering (left-right)

def get_throttle_input():
    throttle = joystick.get_axis(1)
    #print(f"Throttle: {min(0.0, -throttle): .2f}")
    return max(0.0, -throttle)  # Axis 1 for throttle (forward/backward)

def get_brake_input():
    brake = joystick.get_axis(2)
    #print(f"Brake: {max(0.0, -brake): .2f}")
    return max(0.0, -brake)  # Axis 2 for brake (left-right)




# Connect to CARLA
def connect_to_carla():
    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        #world = client.load_world('Town01')
        logging.info("Successfully connected to CARLA.")
        return world
    except carla.ClientError as e:
        logging.error(f"Failed to connect to CARLA: {e}")
        return None

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

    CAMERA_POS_Z = 3
    CAMERA_POS_X = -5
    camera_init_trans = carla.Transform(carla.Location(z = CAMERA_POS_Z, x = CAMERA_POS_X))

    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

    def camera_callback(image, data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    
    depth_camera_bp = bp_lib.find('sensor.camera.depth')
    depth_camera = world.spawn_actor(depth_camera_bp, camera_init_trans, attach_to = vehicle)

    def depth_callback(image, data_dict):
        image.convert(carla.ColorConverter.LogarithmicDepth)
        data_dict['depth_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    
    VIRIDIS = np.array(matplotlib.colormaps.get_cmap('plasma').colors)
    VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

    COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
    COOL = np.array(matplotlib.colormaps.get_cmap('winter')(COOL_RANGE))
    COOL = COOL[:,:3]
    
    def add_open3d_axis(vis):
        ''' Add a small 3D axis on Open3D visualizer '''
        axis = o3d.geometry.LineSet()
        axis.points = o3d.utility.Vector3dVector(np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
        axis.lines = o3d.utility.Vector2iVector(np.array([
            [0, 1],
            [0, 2],
            [0, 3]]))
        axis.colors = o3d.utility.Vector3dVector(np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
        vis.add_geometry(axis)

    lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '100.0')
    lidar_bp.set_attribute('noise_stddev', '0.1')
    lidar_bp.set_attribute('upper_fov', '15.0')
    lidar_bp.set_attribute('lower_fov', '-25.0')
    lidar_bp.set_attribute('channels', '64.0')
    lidar_bp.set_attribute('rotation_frequency', '20.0')
    lidar_bp.set_attribute('points_per_second', '500000')

    lidar_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z, x=CAMERA_POS_X))
    lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to = vehicle)

    def lidar_callback(point_cloud, point_list):
        ''' Prepares a point cloud with intensity colors ready to be consumed by Open3D'''
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype = np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the intensity and compute a color for it
        intensity = data[:, -1]
        intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
        int_color = np.c_[
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

        points = data[:, :-1]
        points[:, :1] = -points[:, :1]

        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)
    
    point_list = o3d.geometry.PointCloud()
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    camera_data = {'image' : np.zeros((image_h, image_w, 4)), 'depth_image': np.zeros((image_h, image_w, 4))}
    camera.listen(lambda image: camera_callback(image, camera_data))
    lidar.listen(lambda data: lidar_callback(data, point_list))   
    depth_camera.listen(lambda image: depth_callback(image, camera_data))

    cv2.namedWindow('RGB Camera', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Depth Camera', cv2.WINDOW_NORMAL)

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name = 'Carla Lidar',
        width = 960,
        height = 540,
        left = 480,
        top = 270)

    vis.get_render_option().background_color = [0.05, 0.05, 0.05]
    vis.get_render_option().point_size = 1
    vis.get_render_option().show_coordinate_frame = True
    add_open3d_axis(vis)
    frame = 0
    # Step 4: Start the game loop and handle input
    try:
        while True:
            pygame.event.pump()  # Process any pygame events (important for joystick)

            # Step 5: Read G29 inputs
            steering = get_steering_input()
            throttle = get_throttle_input()
            brake = get_brake_input()
           
            # Step 6: Apply the controls to the vehicle
            control_vehicle(vehicle, steering, throttle, brake)
            # Update the display and check for the quit event
            # cv2.imshow('RGB Camera', sensor_data['image'])
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_geometry(point_list)
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.005)
            frame +=1
            
            cv2.imshow('RGB Camera', camera_data['image'])
            cv2.imshow('Depth Camera', camera_data['depth_image'])
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()
        lidar.stop()
        lidar.destroy()
        camera.stop()
        camera.destroy()
        depth_camera.stop()
        depth_camera.destroy()
            # Step 7: Sleep for a short time to allow the vehicle to respond
            #time.sleep(0.01)

           
    except KeyboardInterrupt:
        logging.info("Simulation ended.")
    finally:
        # Clean up by destroying the vehicle actor
        vehicle.destroy()

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
