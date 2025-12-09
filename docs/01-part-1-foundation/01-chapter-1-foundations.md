---
id: chapter-1-foundations
title: "Chapter 1: Foundations of Physical AI"
sidebar_label: "1. Foundations of Physical AI"
---

# Foundations of Physical AI and Embodied Intelligence

> "Intelligence isn't just in the brain; it's in the interaction between the brain, the body, and the world."

## 1.1 Defining Physical AI and Embodied Intelligence

**Physical AI** refers to artificial intelligence systems that interact directly with the physical world through sensors and actuators. Unlike **Digital AI** (e.g., ChatGPT, image classifiers), which operates in the realm of bits and pixels, Physical AI must contend with the immutable laws of physics—gravity, friction, inertia, and collisions.

**Embodied Intelligence** is the concept that intelligence emerges from the interplay between an agent's control system (neural interface) and its physical morphology (body). A humanoid robot's ability to walk isn't just code; it's also the mechanical design of its legs and the physics of the environment.

### The Great Divide: Digital vs. Physical AI

| Feature | Digital AI | Physical AI |
| :--- | :--- | :--- |
| **Environment** | Static, structured data (Text, Images) | Dynamic, unstructured physical world |
| **Cost of Failure** | Low (Incorrect prediction) | High (Hardware damage, safety risks) |
| **Feedback Loop** | Open-loop or offline training | Closed-loop, real-time control (ms latency) |
| **Data Source** | Internet-scraped datasets | Real-world sensory streams (noisy, incomplete) |

## 1.2 The Humanoid Robotics Landscape

Why humanoids? The world is designed for humans. Stairs, door handles, tools, and vehicles are engineered for the human form factor. A general-purpose robot must therefore navigate this human-centric environment.

### Key Platforms in this Course
We will reference specific hardware to ground our theory:
*   **Unitree G1/Go2**: Advanced quadruped/humanoid platforms for locomotion research.
*   **NVIDIA Jetson Orin Nano**: The edge AI computer serving as the robot's brain.
*   **RTX-Enabled Workstation**: For high-fidelity digital twin simulation (Isaac Sim).

:::info[Why Form Matters]
A wheeled robot cannot climb a ladder. A robotic arm bolted to a table cannot fetch coffee from the kitchen. The **humanoid form** unlocks the potential for truly general-purpose utility.
:::

## 1.3 Anatomy of a Robot: Components and Subsystems

A robot is a system of systems. We can break it down into three main categories: Actuation (Muscles), Perception (Senses), and Computation (Brain).

### 1.3.1 Actuators (The Muscles)
Actuators convert energy into motion.
*   **DC Motors**: High RPM, low torque. Requires gearing.
*   **BLDC (Brushless DC) Motors**: The standard for modern robotics (e.g., Unitree motors). High efficiency, precise control.
*   **Series Elastic Actuators (SEA)**: Motors with integrated springs for shock absorption and force control.

### 1.3.2 Sensors (The Senses)
Sensors provide the feedback loop necessary for intelligence.

1.  **LiDAR (Light Detection and Ranging)**:
    *   *Function*: Shoots laser pulses to measure distance. Creates a 3D point cloud of the world.
    *   *Role*: Accurate obstacles detection and mapping (SLAM).
2.  **Cameras (RGB & Depth)**:
    *   *Hardware*: **Intel RealSense D435i**.
    *   *Function*: Provides color (RGB) for semantic understanding and depth (D) for geometry.
3.  **IMU (Inertial Measurement Unit)**:
    *   *Function*: The inner ear of the robot. Measures acceleration (accelerometer) and rotation rate (gyroscope).
    *   *Critical for*: Balance and stabilization.
4.  **Force/Torque Sensors**:
    *   *Function*: Measures interaction forces at the feet or hands. Essential for compliant manipulation.

### 1.3.3 Compute (The Brain)
*   **Edge Compute**: Onboard the robot (e.g., **Jetson Orin Nano**). Handles real-time control, sensor fusion, and safety.
*   **Cloud/Workstation**: Offboard (e.g., **RTX 4090 PC**). Handles heavy training, massive simulations, and LLM inference.

## 1.4 Sim-to-Real: The Development Pipeline

Developing directly on hardware is slow and expensive. We use a **Sim-to-Real** workflow:
1.  **Design**: Create robot URDF/CAD.
2.  **Train**: Use Reinforcement Learning (RL) in simulation (Isaac Sim/Gazebo).
3.  **Transfer**: Deploy the trained policy to the physical robot (Unitree G1).
4.  **Verify**: Test in the real world with safety layers.

## Summary
Physical AI bridges the gap between digital intelligence and the physical world. By understanding the hardware constraints—actuators, sensors, and compute—we can build software that doesn't just think, but *acts*.
