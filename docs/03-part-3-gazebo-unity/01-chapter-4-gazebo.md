---
id: chapter-4-gazebo-simulation
title: "Chapter 4: The Digital Twin - Gazebo"
sidebar_label: "4. Gazebo & Physics"
---

# The Digital Twin: Gazebo and Physics Simulation

## 4.1 Introduction to Robot Simulation

Before we let a 50kg robot loose in a lab, we test it in a **Digital Twin** environment. A simulator solves the equations of motion (Newton-Euler) to predict how the robot will behave under gravity, contact, and friction.

### Why Gazebo?
**Gazebo** is the standard simulator for ROS 2.
*   **Physics Engine**: Supports ODE, Bullet, Simbody, and DART.
*   **Sensor Simulation**: Can simulate cameras, LiDARs, and IMUs.
*   **ROS 2 Integration**: Seamless. It publishes sensor topics just like real hardware.

## 4.2 Setting Up the Environment

Installation is usually done via `ros-humble-gazebo-ros-pkgs` (or `ros-jazzy-...`).

### Spawning a Robot
To spawn a robot, we use a launch file that:
1.  Starts the Gazebo server (`gzserver`) and client (`gzclient`).
2.  Parses the URDF file.
3.  Calls the `spawn_entity` service.

```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_humanoid
```

## 4.3 Physics Engines and Rigid Body Dynamics

To trust the sim, you must understand the physics.

### 4.3.1 Rigid Body Dynamics
The simulator treats links as **Rigid Bodies** (non-deformable). It calculates:
*   **Gravity**: F = mg acting on the Center of Mass (CoM).
*   **Coriolis & Centrifugal Forces**: Effects of rotation.
*   **Inertia**: Resistance to rotational acceleration.

### 4.3.2 Collisions
Defining collisions correctly is critical.
*   **Meshes vs. Primitives**: Using a complex mesh (10k triangles) for collision is slow. Use primitives (box, cylinder, sphere) to approximate the shape for fast calculation.
*   **Friction**: The coefficient of friction ($\mu$) determines if your robot slips or grips the floor.
    *   *Ice*: μ ≈ 0.05
    *   *Rubber on Carpet*: μ ≈ 1.0

## 4.4 URDF to SDF

While ROS uses URDF, Gazebo uses **SDF (Simulation Description Format)**.
*   **URDF**: Tree structure (Robot -> Link -> Joint).
*   **SDF**: World structure (World -> Light -> Model -> Link).

ROS 2 automatically converts URDF to SDF when spawning, but you can add Gazebo-specific tags to your URDF to control color, friction, and plugins.

```xml
<gazebo reference="foot_link">
  <material>Gazebo/Red</material>
  <mu1>0.9</mu1> <!-- Friction coefficient -->
  <mu2>0.9</mu2>
</gazebo>
```

## 4.5 Workshop: Spawning a Humanoid in a World

We will create a simple world file `my_world.world` with a ground plane and a sun.

1.  **Create World File**:
    ```xml
    <sdf version="1.6">
      <world name="default">
        <include><uri>model://ground_plane</uri></include>
        <include><uri>model://sun</uri></include>
      </world>
    </sdf>
    ```

2.  **Launch with Empty World**:
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py world:=path/to/my_world.world
    ```

3.  **Spawn Robot**:
    Watch your URDF collapse into a pile of parts on the floor! (Because we haven't written the control software yet).
