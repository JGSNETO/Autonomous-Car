# CARLA Joystick-Controlled Vehicle with RGB, Depth, and LiDAR Visualization

This project connects to the CARLA simulator, spawns a Tesla Model 3, and allows real-time driving control using a Logitech G29 racing wheel. It visualizes camera and LiDAR data live using OpenCV and Open3D.

---

## Features

- Connects to CARLA server (`localhost:2000`)
- Loads `Town01` map
- Spawns Tesla Model 3
- Attaches RGB camera, depth camera, and LiDAR sensors
- Reads Logitech G29 inputs (steering, throttle, brake)
- Displays:
    - RGB camera in OpenCV window
    - Depth camera in OpenCV window
    - LiDAR point cloud in Open3D window

---

# Requirements

- Python 3.8
- CARLA Python API
- pygame
- numpy
- opencv-python
- open3d
- matplotlib

## Install dependencies:
pip install pygame numpy opencv-python open3d matplotlib


---

## How to Run

1. Start the CARLA simulator: ./CarlaUE4.sh

2. Run the script: python control_vehicle.py


3. Control the car using the Logitech G29 wheel.

4. Press `q` in the OpenCV windows to quit.

---

## Controls (G29 Joystick)

| Control    | Axis/Action       |
|------------|-------------------|
| Steering   | Axis 0            |
| Throttle   | Axis 1 (negative) |
| Brake      | Axis 2 (negative) |

---

## Code Overview

This script:

- Initializes pygame for joystick input.
- Connects to CARLA (`localhost:2000`) and loads `Town01`.
- Spawns a Tesla Model 3.
- Attaches:
    - RGB camera (`sensor.camera.rgb`)
    - Depth camera (`sensor.camera.depth`)
    - LiDAR (`sensor.lidar.ray_cast`)
- Reads real-time inputs from the Logitech G29:
    - Axis 0 → Steering
    - Axis 1 → Throttle (inverted)
    - Axis 2 → Brake (inverted)
- Displays:
    - Camera feeds (OpenCV windows)
    - LiDAR point cloud (Open3D visualizer)
- Cleans up actors and windows on exit.

---

## Main Functions

- **connect_to_carla()**: Connects to CARLA server and loads the map.
- **control_vehicle()**: Applies steering, throttle, and brake inputs to the vehicle.
- **camera_callback()**: Processes RGB camera data.
- **depth_callback()**: Processes depth camera data.
- **lidar_callback()**: Processes LiDAR point cloud data.

---

## Cleanup

On exit (`q` or interrupt):
- Stops and destroys all sensors.
- Destroys the vehicle actor.
- Closes OpenCV and Open3D windows.

---

## Notes

✅ Make sure the CARLA server is running before executing the script.  
✅ Adjust the map (`Town01`) or vehicle blueprint (`vehicle.tesla.model3`) if needed.  
✅ Ensure the G29 wheel is properly connected and detected by pygame.



