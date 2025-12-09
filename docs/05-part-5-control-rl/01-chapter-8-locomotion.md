---
id: chapter-8-locomotion
title: "Chapter 8: Humanoid Locomotion"
sidebar_label: "8. Locomotion & Navigation"
---

# Humanoid Locomotion and Navigation

## 8.1 The Challenge of Bipedalism

Walking is controlled falling. The **Inverted Pendulum** model describes bipedal dynamics. The Center of Mass (CoM) must be kept over the Base of Support (BoS).

### Zero Moment Point (ZMP)
The ZMP is the point on the ground where the tipping moment is zero. If the ZMP stays within the polygon of the feet (Support Polygon), the robot is stable.

## 8.2 Reinforcement Learning (RL) for Locomotion

Modern humanoids (Unitree, Tesla Optimus) use **RL** instead of classical ZMP control.
*   **Policy**: A neural network that takes `State` (joints, IMU) and outputs `Action` (motor targets).
*   **Reward Function**:
    ```python
    reward = (vel_x - target_vel_x) - (energy_penality) - (stumble_penality)
    ```
*   **Training**: We train this in **Isaac Lab** with millions of steps in minutes.

## 8.3 Navigation 2 Stack (Nav2)

Once the robot can "walk" (low-level control), it needs to "go" somewhere (high-level planning). **Nav2** is the industry standard navigation stack.

### Components
1.  **Planner (Global)**: Dijkstra/A* algorithm. Finds a path from A to B on the map.
2.  **Controller (Local)**: Follows the path while avoiding dynamic obstacles (people, dogs).
    *   *For Humanoids*: We use **MPPI (Model Predictive Path Integral)** control, which understands the non-holonomic constraints of walking.

## 8.4 Manipulation and Grasping

Locomotion gets you there; manipulation does the work.

### MoveIt 2
**MoveIt** is the motion planning framework for arms.
1.  **Planning Group**: Define the arm chain (shoulder to wrist).
2.  **OMPL**: The Open Motion Planning Library solves the IK path to avoid self-collisions.

```python
# Moving arm to a pose using Python MoveIt Interface
move_group.set_pose_target(target_pose)
plan = move_group.plan()
move_group.execute(plan)
```

## 8.5 Integrating Walking and Manipulation

**Whole-Body Control (WBC)** coordinates the legs and arms.
*   *Example*: To reach a high shelf, the robot effectively "tiptoes" (extends legs) while reaching (extends arm).
*   *Priority*: Balance > Walking > Reach.

In the next chapter, we give the robot a mind to control this body.
