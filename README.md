# CARLA Manual Control Simulation with Sensor Visualization

This script implements a manual control simulation for the CARLA simulator using a physical joystick (e.g., Logitech G29). It includes visualization for both RGB camera and LiDAR data using OpenCV and Open3D.

---

## 📦 Required Modules
- `pygame`: Joystick input handling
- `carla`: CARLA simulator client API
- `open3d`: Real-time 3D visualization of LiDAR data
- `cv2 (OpenCV)`: Display of RGB camera feed
- `numpy`: Matrix and array operations
- `matplotlib`: Color mapping for LiDAR intensities

---

## 🧱 Main Components

### `SensorManager`
Manages RGB camera and LiDAR sensors.

#### Constructor:
```python
SensorManager(world, vehicle, camera_pos_x=0, camera_pos_z=3, image_width=650, image_height=360, lidar_pos_x=0, lidar_pos_z=3)
```

#### Key Methods:
- `_spawn_rgb_camera()`: Spawns the RGB camera sensor
- `_listen_to_camera()`: Sets up callback for camera images
- `_camera_callback(image)`: Stores incoming image data
- `get_image()`: Returns the current camera frame
- `_spawn_lidar()`: Spawns the LiDAR sensor with specified attributes
- `_listen_to_lidar()`: Sets up callback for LiDAR point clouds
- `_lidar_callback(data)`: Processes LiDAR point cloud and assigns color based on intensity
- `_setup_open3d_visualizer()`: Creates Open3D visualizer window
- `_add_open3d_axis()`: Adds coordinate axis to the visualizer
- `update_visualizations()`: Refreshes the visualizer with new point cloud data

---

### `JoystickHandler`
Interfaces with a joystick device for manual driving control.

#### Constructor:
```python
JoystickHandler(vehicle)
```

#### Key Methods:
- `get_steering_input()`: Returns steering axis value
- `get_throttle_input()`: Returns throttle axis value
- `get_brake_input(vehicle)`: Returns brake value and updates brake light

---

### `VehicleManager`
Spawns and controls a vehicle actor in CARLA.

#### Constructor:
```python
VehicleManager(world, vehicle_blueprint='vehicle.tesla.model3')
```

#### Key Methods:
- `_spawn_vehicle(blueprint_id)`: Spawns the vehicle at a spawn point
- `enable_brake_lights(state)`: Enables or disables brake lights
- `apply_control(throttle, steer, brake)`: Applies manual driving input to the vehicle

---

## 🎮 `game_loop(args)`
Main execution loop:
- Connects to CARLA server
- Spawns vehicle and attaches sensors
- Reads joystick input for throttle, steering, and brake
- Updates vehicle control and sensor visualizations in real-time
- Press **Q** to exit the simulation

---

## 🏁 `main()`
Parses command-line arguments and starts the simulation via `game_loop`.

---

## ▶️ To Run the Script
```bash
python your_script.py
```

Optionally use:
```bash
python your_script.py --sync --verbose
```

---

## 🧹 Cleanup
- All actors are destroyed on shutdown
- Visualization windows are closed properly
