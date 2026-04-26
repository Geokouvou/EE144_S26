#!/bin/bash
set -e
echo "=== EE144 Lab 2 setup ==="

# 1) Copy beacons world into the place Gazebo's launch file looks
sudo cp ~/workspace/ros2_ws/src/ee144_pkg/worlds/beacons.sdf \
    /opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds/
echo "[1/2] World copied."

# 2) Patch the Turtlebot4 spawn launch to skip spawning the dock
SPAWN_LAUNCH=/opt/ros/jazzy/share/turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py
if grep -q "DISABLED for EE144 lab" "$SPAWN_LAUNCH"; then
  echo "[2/2] Dock spawn already disabled."
else
  sudo python3 << 'PYEOF'
path = '/opt/ros/jazzy/share/turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py'
with open(path) as f:
    text = f.read()

old1 = """        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dock_description_launch]),
            # The robot starts docked
            launch_arguments={'gazebo': 'ignition'}.items(),
        ),"""
new1 = """        # Dock description -- DISABLED for EE144 lab
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([dock_description_launch]),
        #     launch_arguments={'gazebo': 'ignition'}.items(),
        # ),"""

old2 = """        # Spawn Dock
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', dock_name,
                       '-x', x_dock,
                       '-y', y_dock,
                       '-z', z,
                       '-Y', yaw_dock,
                       '-topic', 'standard_dock_description'],
            output='screen',
        ),"""
new2 = """        # Spawn Dock -- DISABLED for EE144 lab
        # Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     arguments=['-name', dock_name,
        #                '-x', x_dock,
        #                '-y', y_dock,
        #                '-z', z,
        #                '-Y', yaw_dock,
        #                '-topic', 'standard_dock_description'],
        #     output='screen',
        # ),"""

text = text.replace(old1, new1).replace(old2, new2)

with open(path, 'w') as f:
    f.write(text)
PYEOF
  echo "[2/2] Dock spawn disabled."
fi

echo "=== Setup complete. ==="
echo ""
echo "Now launch with:"
echo "  ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite world:=beacons"
