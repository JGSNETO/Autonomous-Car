import carla
import cv2
import math
import random
import time
import numpy as np
import logging

logging.basicConfig(level=logging.INFO)
def connect_to_carla():
    """Connect to CARLA server and return the world object."""
    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        logging.info("Successfully connected to CARLA.")
        return world
    except carla.ClientError as e:
        logging.error(f"Failed to connect to CARLA server: {e}")
    except Exception as e:
        logging.error(f"An error occurred: {e}")

def get_blueprints(world):
    """Get blueprint library from the world."""
    try:
        bp_lib = world.get_blueprint_library()
        logging.info("Successfully loaded blueprints.")
        return bp_lib
    except Exception as e:
        logging.error(f"An error occurred while loading blueprints: {e}")

def get_spawnpoints(world):
    """Get the spawn points from the world map."""
    try:
        spawn_points = world.get_map().get_spawn_points()
        logging.info("Successfully loaded spawn points.")
        return spawn_points
    except Exception as e:
        logging.error(f"An error occurred while loading spawn points: {e}")

def load_vehicle(world, spawn_points, vehicle):
    try:
        vehicle_blueprint = world.get_blueprint_library().filter(vehicle)
        vehicle = world.try_spawn_actor(vehicle_blueprint[0], spawn_points[0])
        logging.info("Successfully loaded the vehicle.")
        return vehicle
    except Exception as e:
        logging.error(f"An error occurred while loading the vehicle.")

def set_autopilot(vehicle):
    try:
        vehicle.set_autopilot(True)
        logging.info("Successfully set the autopilot in the vehicle.")
        return True
    except Exception as e:
        logging.error(f"An error occurred while setting the autopilot.")
        return None


def destroy_all_actors(world):
    try:
        for actor in world.get_actors().filter('*vehicle*'):
            actor.destroy()
        logging.info('Successfully destroyed all actors.')
        return True
    except Exception as e:
        logging.error(f'An error occurred while destroying all actors.')
        return None
    
def main():

    """" Main Function to run the program"""
    logging.info("Program started")

    #Step 1: Connect to CARLA
    world = connect_to_carla()
    if not world:
        logging.error("Exiting program due to connection failure.")
        return 

    # Step 2: Get blueprints 
    blueprints = get_blueprints(world)
    if not blueprints:
        logging.error("Exiting program due to failure in loading blueprints.")
        return 
    
    # Step 3: Get spawn points
    spawn_points = get_spawnpoints(world)
    if not spawn_points:
        logging.error("Exiting program due to failure in loading the spawn points.")
    
    # Step 4: Load the vehicle
    vehicle = load_vehicle(world, spawn_points, "cybertruck")
    if not vehicle:
        logging.error("Exiting program due to failure in loading the vehicle.")

    # Step 5: Set the autopilot
    autopilot = set_autopilot(vehicle)
    if not autopilot:
        logging.error("Exiting the program due to failure in setting the autopilot")
    # # Step 6: Destroy all actors
    # destroy = destroy_all_actors(world)
    # if not destroy:
    #     logging.error("Exiting the program due to failure in destroying the actors.")

    logging.info("Program completed successfully.")

if __name__ == "__main__":
    main()