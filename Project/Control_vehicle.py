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
import argparse
import numpy as np
import carla

VIRIDIS = np.array(matplotlib.colormaps.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

class SensorManager:
    def __init__(self, world, vehicle, camera_pos_x=0, camera_pos_z=3, image_width=650, image_height=360, lidar_pos_x=0, lidar_pos_z=3):
        # Simulation setup
        self.world = world
        self.vehicle = vehicle
        # Camera setup
        self.camera_pos_x = camera_pos_x
        self.camera_pos_z = camera_pos_z
        self.image_width = image_width
        self.image_height = image_height
        self.camera_data = {'image': np.zeros((self.image_height, self.image_width, 4), dtype=np.uint8)}
        #Spawn RGB camera
        self.camera = self._spawn_rgb_camera()
        self._listen_to_camera()
        # LIDAR Setup
        self.lidar_pos_x = lidar_pos_x
        self.lidar_pos_z = lidar_pos_z
        self.point_list = o3d.geometry.PointCloud()
        # Open3D visualizer setup
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='CARLA LiDAR', width=960, height=540, left=480, top=270)
        self.vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        self.vis.get_render_option().point_size = 1.0
        self.vis.get_render_option().show_coordinate_frame = True
        self._add_open3d_axis()
        self.vis.add_geometry(self.point_list)
        # Spawn LIDAR sensor
        self.lidar = self._spawn_lidar()
        self._listen_to_lidar()

    def _spawn_rgb_camera(self):
        bp_lib = self.world.get_blueprint_library()
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.image_width))
        camera_bp.set_attribute('image_size_y', str(self.image_height))

        camera_init_trans = carla.Transform(carla.Location(z=self.camera_pos_z, x=self.camera_pos_x))

        # Pass attach_to as positional argument
        return self.world.spawn_actor(
            camera_bp,
            camera_init_trans,
            self.vehicle,  # positional attach_to
            attachment_type=carla.AttachmentType.Rigid  # keyword argument
        )
    
    #--------------------Camera--------------------
    
    def _listen_to_camera(self):
        self.camera.listen(lambda image: self._camera_callback(image))
    
    def _camera_callback(self, image):
        self.camera_data['image'] = np.reshape(
            np.copy(image.raw_data),
            (image.height, image.width, 4)
        )
    
    def get_image(self):
        return self.camera_data['image']

    #--------------------LIDAR--------------------
    def _spawn_lidar(self):
        bp_lib = self.world.get_blueprint_library()
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '100.0')
        lidar_bp.set_attribute('noise_stddev', '0.1')
        lidar_bp.set_attribute('upper_fov', '15.0')
        lidar_bp.set_attribute('lower_fov', '-25.0')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('points_per_second', '500000')

        lidar_init_trans = carla.Transform(carla.Location(z=self.lidar_pos_z, x=self.lidar_pos_x))
        return self.world.spawn_actor(lidar_bp, lidar_init_trans, self.vehicle, attachment_type=carla.AttachmentType.Rigid)
    
    def _listen_to_lidar(self):
        self.lidar.listen(lambda data: self._lidar_callback(data))

    def _lidar_callback(self, point_cloud):
        data = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
        intensity = data[:, 3]
        intensity_col = 1.0 - np.log(intensity + 1e-8) / np.log(np.exp(-0.004 * 100))
        intensity_col = np.clip(intensity_col, 0.0, 1.0)

        int_color = np.c_[
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

        #points = data[:, :3]
        points = np.copy(data[:, :-1])
        points[:, 0] = -points[:, 0]

        self.point_list.points = o3d.utility.Vector3dVector(points)
        self.point_list.colors = o3d.utility.Vector3dVector(int_color)

    # ---------------- VISUALIZATION ----------------

    def _setup_open3d_visualizer(self):
        self.vis.create_window(
            window_name='CARLA LiDAR',
            width=960,
            height=540,
            left=480,
            top=270
        )
        self.vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        self.vis.get_render_option().point_size = 1
        self.vis.get_render_option().show_coordinate_frame = True
        self._add_open3d_axis()

    def _add_open3d_axis(self):
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
        self.vis.add_geometry(axis)
    
    def update_visualizations(self):
    # Update LiDAR point cloud in Open3D
        self.vis.update_geometry(self.point_list)
        self.vis.poll_events()
        self.vis.update_renderer()

class JoystickHandler:
    def __init__(self, vehicle):
        pygame.init()
        pygame.joystick.init()
        self.vehicle = vehicle

        if pygame.joystick.get_count() == 0:
            logging.error("No joystick detected.")
            self.joystick = None
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logging.info(f"Joystick initialized: {self.joystick.get_name()}")

    def get_steering_input(self) -> float:
        if self.joystick:
            return self.joystick.get_axis(0)  # Steering axis
        return 0.0

    def get_throttle_input(self) -> float:
        if self.joystick:
            #print(self.joystick.get_axis(1))
            return max(0.0, -self.joystick.get_axis(1))  # Throttle axis
        return 0.0

    def get_brake_input(self, vehicle) -> float:
        if self.joystick:
            #return max(0.0, -self.joystick.get_axis(2))
            if max(0.0, -self.joystick.get_axis(2)) > 0.1:
                self.vehicle.enable_brake_lights(True)
                return max(0.0, -self.joystick.get_axis(2))  # Brake axis
            else:
                self.vehicle.enable_brake_lights(False)
                return max(0.0, -self.joystick.get_axis(2))  # Brake axis
        return 0.0

class VehicleManager:
    def __init__(self, world, vehicle_blueprint: str = "vehicle.tesla.model3"):
        self.world = world
        self.vehicle = self._spawn_vehicle(vehicle_blueprint)
        self.light_state = carla.VehicleLightState.NONE

    def _spawn_vehicle(self, blueprint_id):
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find(blueprint_id)
        spawn_point = self.world.get_map().get_spawn_points()[1]
        return self.world.spawn_actor(vehicle_bp, spawn_point)
    
    def enable_brake_lights(self, light_state):
        # Set the light state to include the Brake light
        if light_state == True:
            self.light_state = carla.VehicleLightState.Brake
        else:
            self.light_state = carla.VehicleLightState.NONE
        self.vehicle.set_light_state(carla.VehicleLightState(self.light_state))
    
    def apply_control(self, throttle: float, steer: float, brake: float):
        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
         # Ensure vehicle is in drive gear before moving 
        if self.vehicle.get_control().gear != 1:  # Gear 1 is Drive mode
            control.gear = 1  # Set the gear to Drive mode
        self.vehicle.apply_control(control)



def game_loop(args):
    pygame.init()

    try:
        #client = carla.Client(args.host, args.port)
        client = carla.Client('localhost', 2000)
        client.set_timeout(2000.0)
        sim_world = client.get_world()
        vehicle = VehicleManager(sim_world)
        Joystick_handler = JoystickHandler(vehicle)
        sensor_manager = SensorManager(sim_world, vehicle.vehicle, -5, 2)
        frame = 0
        added_lidar = False

        while True:
            pygame.event.pump()
            # Step 5: Read G29 inputs
            steering = Joystick_handler.get_steering_input()
            throttle = Joystick_handler.get_throttle_input()
            brake = Joystick_handler.get_brake_input(vehicle)
            vehicle.apply_control(throttle, steering, brake)
            sim_world.tick()
            frame += 1
            # Small sleep to match real-time speed (optional)
            time.sleep(0.05)

            if cv2.waitKey(1) == ord('q'):
                break

            image = sensor_manager.get_image()
            if image is not None:
                cv2.imshow('Camera feed', image)
            
            # LiDAR visualization
            if not added_lidar and frame == 2:
                sensor_manager.vis.add_geometry(sensor_manager.point_list)
                added_lidar = True
            sensor_manager.update_visualizations()

        cv2.destroyAllWindows()
        for actor in sim_world.get_actors().filter('*vehicle*'):
            actor.destroy()
        for sensor in sim_world.get_actors().filter('*sensor*'):
            sensor.destroy()
    
    except KeyboardInterrupt:
        logging.info("Simulation ended.")
    
# ---------------------------------------------------
# -- main() -----------------------------------------
# ---------------------------------------------------
def main():
    argparser = argparse.ArgumentParser(
        description = 'CARLA Manual Control Client'
    )
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information'
    )
    argparser.add_argument(
        '--sync',
        action='store_true',
        help = 'Activate synchronous mode execution'
    )
    args = argparser.parse_args()

    #return args

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print("\nCancelled by user.")

if __name__ == '__main__':
    main()