# ros2_ws (Joystick only)

## Build
```bash
cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch robot_bringup bringup.launch.py port:=/dev/ttyUSB0 baud:=115200
```

## Notes
- Requires packages: `joy`, `geometry_msgs`, `sensor_msgs`, `rclpy`, `pyserial`.
- Joystick: hold LB to enable, RB for turbo (configurable in params).
- ESP32 side should accept lines like `VW V=0.500 W=0.000` over serial.
