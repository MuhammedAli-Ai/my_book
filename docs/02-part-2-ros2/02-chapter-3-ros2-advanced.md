---
id: chapter-3-ros2-advanced
title: "Chapter 3: ROS 2 Advanced"
sidebar_label: "3. ROS 2: Advanced Control"
---

# ROS 2: Advanced Control and Description

## 3.1 Managing Complexity: Launch Files

Starting nodes one by one (`ros2 run ...`) is impossible for a humanoid with 50+ processes. **Launch files** automate this.

Written in Python (or XML/YAML), launch files allow you to:
1.  Start multiple nodes at once.
2.  Configure parameters (e.g., change simulation vs. real hardware modes).
3.  Manage node lifecycles.

### Example Launch File
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='heartbeat_node',
            name='heartbeat'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[{'video_device': '/dev/video0'}]
        )
    ])
```

## 3.2 URDF: The Body Schema

The **Unified Robot Description Format (URDF)** is an XML file that describes the physical properties of your robot. Without a URDF, ROS doesn't know what your robot looks like.

### Key Elements
*   **Links**: The rigid parts (bones). E.g., `tibia`, `femur`, `torso`.
*   **Joints**: The moving parts (connectors). E.g., `knee_joint` (revolute), `hip_joint` (spherical).
*   **Visual**: Mesh files (.dae/.stl) for looking good in visualization.
*   **Collision**: Simplified geometry (boxes/cylinders) for physics engines to calculate contacts efficiently.
*   **Inertial**: Mass and moments of inertia. Critical for dynamic simulation.

### Anatomy of a User-Created Humanoid Link
```xml
<link name="upper_leg">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/upper_leg.stl"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

:::warning[Garbage In, Garbage Out]
If your `inertial` values are wrong (e.g., mass is 1kg but real robot is 5kg), your simulation will be worthless, and your control algorithms will fail on the real robot.
:::

## 3.3 Kinematics and Dynamics

To control a humanoid, we need math.

### 3.3.1 Forward Kinematics (FK)
*   *Question*: "If I set my hip angle to 30° and knee to 45°, where is my foot?"
*   *Process*: Joint Angles -> End Effector Position.

### 3.3.2 Inverse Kinematics (IK)
*   *Question*: "I want my foot to step at (x=0.5, y=0.1, z=0.0). What angles do the hip and knee need to be?"
*   *Process*: End Effector Position -> Joint Angles.
*   *Challenge*: Harder to solve. Multiple solutions often exist (elbow up vs. elbow down).

### 3.3.3 Dynamics
*   *Question*: "How much torque does the motor need to lift this leg against gravity?"
*   *Tool*: Uses the URDF mass/inertia data.

## 3.4 Visualizing with Rviz2

**Rviz2** is the window into the robot's mind. It visualizes:
*   The URDF model.
*   Real-time sensor data (LiDAR points, Camera feeds).
*   TF (Transform) trees: The coordinate frames of every joint.

In the next chapter, we will take this static description and make it move in a physics simulator.
