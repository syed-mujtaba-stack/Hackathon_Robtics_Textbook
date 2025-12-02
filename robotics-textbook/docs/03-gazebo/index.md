---
sidebar_position: 4
---

# Gazebo Simulation

## Introduction to Gazebo

**Gazebo** is an open-source 3D robotics simulator that provides accurate physics simulation, rendering, and sensor simulation for testing robot algorithms in realistic environments.

:::tip Why Simulation?
Testing on real robots is expensive, time-consuming, and risky. Gazebo lets you iterate quickly and safely before deploying to hardware.
:::

## Gazebo Architecture

```
┌──────────────────────────────────────┐
│         Gazebo Client (GUI)          │
├──────────────────────────────────────┤
│        Physics Engine (ODE, Bullet)  │
├──────────────────────────────────────┤
│      Sensor Simulation (Camera,      │
│       LiDAR, IMU, Force/Torque)      │
├──────────────────────────────────────┤
│      Rendering (OGRE/Ignition)       │
└──────────────────────────────────────┘
```

## Key Features

### 1. **Physics Simulation**
- Multiple physics engines (ODE, Bullet, Simbody, DART)
- Realistic friction, collision, and dynamics
- Adjustable gravity and time step

### 2. **Sensor Models**
- RGB/Depth cameras
- LiDAR/Laser scanners
- IMU (Inertial Measurement Unit)
- GPS, contact sensors, force/torque sensors

### 3. **Robot Models (URDF/SDF)**
- Import from CAD (SOLIDWORKS, Fusion 360)
- Programmatic model generation
- Library of pre-built models

## Getting Started

### Installation

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

### Launch Gazebo with ROS 2

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 
                 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot',
                      '-file', '/path/to/robot.sdf']
        )
    ])
```

## Creating a World

### Simple World File (.world)

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Custom object -->
    <model name="box">
      <static>false</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Robot Model (SDF)

### Differential Drive Robot

```xml
<model name="diff_drive_robot">
  <link name="chassis">
    <pose>0 0 0.1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box><size>0.5 0.3 0.1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.5 0.3 0.1</size></box>
      </geometry>
    </visual>
  </link>
  
  <!-- Left Wheel -->
  <link name="left_wheel">
    <pose>0.2 0.2 0.05 1.5707 0 0</pose>
    <collision name="collision">
      <geometry>
        <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
      </geometry>
    </visual>
  </link>
  
  <!-- Joints, Right Wheel, etc. -->
  
  <!-- Differential Drive Plugin -->
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <command_topic>/cmd_vel</command_topic>
    <odometry_topic>/odom</odometry_topic>
  </plugin>
</model>
```

## Gazebo Plugins for ROS 2

### Common Plugins

| Plugin | Purpose |
|--------|---------|
| `gazebo_ros_diff_drive` | Differential drive control |
| `gazebo_ros_camera` | Camera sensor |
| `gazebo_ros_laser` | LiDAR/laser scanner |
| `gazebo_ros_imu` | IMU sensor |
| `gazebo_ros_joint_state_publisher` | Publish joint states |

### Camera Plugin Example

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=rgb/image_raw</remapping>
      <remapping>camera_info:=rgb/camera_info</remapping>
    </ros>
    <camera_name>camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

## Controlling Robots in Gazebo

### Velocity Control (Python)

```python
import rclpy
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
    
    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # m/s forward
        msg.angular.z = 0.2  # rad/s turn
        self.publisher.publish(msg)
```

## Advanced Features

### 1. **World Plugins**
Custom behavior for entire simulation

### 2. **Model Plugins**
Specific model behavior (e.g., conveyor belt)

### 3. **Sensor Plugins**
Custom sensor implementations

### 4. **GUI Plugins**
Custom visualization and controls

## Practical Example: Warehouse Robot

```python
# Launch warehouse simulation
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'warehouse.world'}.items()
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'warehouse_robot',
            '-file', 'warehouse_robot.sdf',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ]
    )
    
    return LaunchDescription([gazebo, spawn_robot])
```

## Debugging Tips

### Visualizing Sensors

```bash
# View camera output
ros2 run rqt_image_view rqt_image_view

# Echo laser scans
ros2 topic echo /scan

# Visualize in RViz
ros2 run rviz2 rviz2
```

### Performance Optimization

- **Reduce physics update rate** for faster simulation
- **Disable GUI** for headless servers
- **Simplify collision geometries**
- **Use level of detail (LOD)** for complex models

## Exercise

### Build a Mobile Robot
1. Create an SDF model for a 4-wheeled robot
2. Add a camera and LiDAR sensor
3. Write a ROS 2 node to navigate a simple maze
4. Visualize sensor data in RViz

### Challenge
Simulate a robotic arm picking and placing objects:
- Use joint controllers
- Implement grasp detection
- Add force/torque sensors to gripper

## Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [SDF Specification](http://sdformat.org/)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

---

**Learning Objectives** ✅
- [ ] Install and launch Gazebo with ROS 2
- [ ] Create world and robot models in SDF
- [ ] Use Gazebo plugins for sensors and actuators
- [ ] Control simulated robots via ROS 2 topics
- [ ] Debug and optimize simulations
