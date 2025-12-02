---
sidebar_position: 3
---

# ROS 2 Fundamentals

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is an open-source middleware framework for building robot applications. It provides tools, libraries, and conventions to simplify the development of complex robotic systems.

:::info Key Improvement
ROS 2 is a complete redesign of ROS 1, addressing real-time performance, security, and multi-robot communication.
:::

## Why ROS 2?

### Advantages over ROS 1
- ✅ **Real-time capable** (with DDS middleware)
- ✅ **Better security** (authentication, encryption)
- ✅ **No single point of failure** (no roscore required)
- ✅ **Multi-robot support** (via DDS discovery)
- ✅ **Production-ready** (used in autonomous vehicles, drones)

## Core Concepts

### 1. Nodes

A **node** is a single process that performs a specific computation.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 2. Topics (Publisher-Subscriber)

Topics enable asynchronous, many-to-many communication.

**Publisher:**
```python
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Robot!'
        self.publisher_.publish(msg)
```

**Subscriber:**
```python
class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### 3. Services (Request-Response)

Services provide synchronous, one-to-one communication.

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_callback)
    
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### 4. Actions (Long-running Tasks)

Actions enable asynchronous, goal-oriented tasks with feedback.

```python
from action_tutorials_interfaces.action import Fibonacci

class ActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', 
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        # Execute long task with feedback
        feedback_msg = Fibonacci.Feedback()
        # ... send periodic feedback ...
        goal_handle.succeed()
        return result
```

## ROS 2 Architecture

```
┌─────────────────────────────────────────────┐
│         Application Layer (Nodes)           │
├─────────────────────────────────────────────┤
│  ROS Client Library (rclpy, rclcpp, etc.)  │
├─────────────────────────────────────────────┤
│            ROS Middleware (rmw)             │
├─────────────────────────────────────────────┤
│    DDS Implementation (Cyclone, FastDDS)    │
└─────────────────────────────────────────────┘
```

## Package Structure

```
my_robot_package/
├── package.xml          # Package metadata
├── setup.py             # Python setup
├── my_robot_package/
│   ├── __init__.py
│   └── node.py          # Node implementation
├── launch/
│   └── robot.launch.py  # Launch file
├── config/
│   └── params.yaml      # Parameters
└── resource/
```

## Launch Files

Launch files start multiple nodes with configuration:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='node1',
            name='custom_node_name',
            parameters=[{'param1': 'value1'}]
        ),
        Node(
            package='my_package',
            executable='node2'
        )
    ])
```

## Quality of Service (QoS)

ROS 2 allows fine-grained control over message delivery:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

self.publisher_ = self.create_publisher(
    String, 'topic', qos_profile)
```

**QoS Policies:**
- **Reliability**: RELIABLE vs. BEST_EFFORT
- **Durability**: VOLATILE vs. TRANSIENT_LOCAL
- **History**: KEEP_LAST vs. KEEP_ALL

## Practical Example: Robot Arm

```python
class RobotArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', 
            self.joint_callback, 10)
        
        # Publish joint commands
        self.cmd_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/command', 10)
        
        # Service for preset positions
        self.srv = self.create_service(
            MoveToPosition, 'move_to_preset', 
            self.move_callback)
    
    def joint_callback(self, msg):
        self.current_position = msg.position
    
    def move_callback(self, request, response):
        # Move to preset position
        trajectory = self.plan_trajectory(request.position_name)
        self.cmd_pub.publish(trajectory)
        response.success = True
        return response
```

## Common ROS 2 Commands

```bash
# List all nodes
ros2 node list

# Show node info
ros2 node info /my_node

# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/String "data: 'Hello'"

# List services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"

# Run a node
ros2 run package_name executable_name
```

## Exercise

### Build Your First Node
1. Create a publisher node that sends robot velocity commands
2. Create a subscriber node that logs the commands
3. Write a launch file to start both nodes

### Challenge
Implement a service that:
- Takes a target position as input
- Returns the estimated time to reach that position
- Publishes progress updates on a topic

## Resources

- [ROS 2 Documentation](https://docs.ros.org)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse](https://discourse.ros.org)

---

**Learning Objectives** ✅
- [ ] Understand nodes, topics, services, and actions
- [ ] Write basic ROS 2 nodes in Python
- [ ] Use launch files to start multiple nodes
- [ ] Configure QoS profiles
- [ ] Navigate the ROS 2 command-line interface
