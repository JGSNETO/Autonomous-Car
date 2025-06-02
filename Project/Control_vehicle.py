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

class JoystickHandler:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

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

    def get_brake_input(self) -> float:
        if self.joystick:
            return max(0.0, -self.joystick.get_axis(2))  # Brake axis
        return 0.0

class VehicleManager:
    def __init__(self, world, vehicle_blueprint: str = "vehicle.tesla.model3"):
        self.world = world
        self.vehicle = self._spawn_vehicle(vehicle_blueprint)

    def _spawn_vehicle(self, blueprint_id):
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find(blueprint_id)
        spawn_point = self.world.get_map().get_spawn_points()[0]
        return self.world.spawn_actor(vehicle_bp, spawn_point)
    
    def apply_control(self, throttle: float, steer: float, brake: float):
       
        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
         # Ensure vehicle is in drive gear before moving 
        if self.vehicle.get_control().gear != 1:  # Gear 1 is Drive mode
            control.gear = 1  # Set the gear to Drive mode
        self.vehicle.apply_control(control)

def game_loop(args):
    pygame.init()
    world = None
    Joystick_handler = JoystickHandler()
    
    try:
        #client = carla.Client(args.host, args.port)
        client = carla.Client('localhost', 2000)
        client.set_timeout(2000.0)
        sim_world = client.get_world()
        vehicle = VehicleManager(sim_world)

        while True:
            pygame.event.pump()
            # Step 5: Read G29 inputs
            steering = Joystick_handler.get_steering_input()
            throttle = Joystick_handler.get_throttle_input()
            brake = Joystick_handler.get_brake_input()
            vehicle.apply_control(throttle, steering, brake)
    
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