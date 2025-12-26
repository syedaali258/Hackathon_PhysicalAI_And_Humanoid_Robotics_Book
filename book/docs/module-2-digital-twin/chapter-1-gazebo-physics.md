---
title: Chapter 1 - Physics Simulation in Gazebo (gravity, collisions)
sidebar_label: Chapter 1 - Gazebo Physics
description: Implementing physics simulation in Gazebo with gravity, collisions, and realistic material properties
---

# Physics Simulation in Gazebo (gravity, collisions)

## Introduction to Gazebo for Physics Simulation

Gazebo is a powerful physics simulation environment that provides accurate modeling of physical laws, making it ideal for robotics simulation. In the context of digital twins for humanoid robots, Gazebo serves as the physics engine that ensures realistic behavior of virtual robots.

### Why Gazebo for Physics?

Gazebo excels at physics simulation because it:

- Provides realistic gravity, collision detection, and material properties
- Supports complex kinematic chains required for humanoid robots
- Offers accurate sensor simulation integrated with physics
- Has strong ROS integration for robotics workflows
- Supports realistic contact dynamics and friction models

## Setting Up Gazebo for Humanoid Robots

### Basic Gazebo World Structure

A typical Gazebo simulation for humanoid robots includes:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include standard physics properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your humanoid robot model will be placed here -->
    <model name="humanoid_robot">
      <!-- Robot definition -->
    </model>
  </world>
</sdf>
```

### Gravity Configuration

Gravity in Gazebo is configured in the physics engine settings. The standard Earth gravity is approximately -9.81 m/s² in the Z direction:

```xml
<physics type="ode">
  <gravity>0 0 -9.81</gravity>
  <!-- Other physics parameters -->
</physics>
```

You can modify gravity to simulate different environments:

- **Moon gravity**: `0 0 -1.62` m/s² (about 1/6 of Earth's gravity)
- **Mars gravity**: `0 0 -3.71` m/s²
- **Zero gravity**: `0 0 0` m/s² (for space robotics)

## Collision Detection in Gazebo

Collision detection is fundamental to realistic physics simulation. In Gazebo, collisions are detected between collision geometries defined for each link:

### Collision Geometries

Common collision geometries include:

- **Box**: `box` with size parameters
- **Sphere**: `sphere` with radius
- **Cylinder**: `cylinder` with radius and length
- **Mesh**: `mesh` for complex shapes

Example collision definition:

```xml
<link name="link_name">
  <collision name="collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <contact>
        <ode>
          <kp>1e+16</kp>
          <kd>1e+13</kd>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

### Contact Parameters

- **kp (Proportional gain)**: Stiffness of the contact
- **kd (Derivative gain)**: Damping of the contact
- **mu and mu2**: Friction coefficients in primary and secondary directions

## Material Properties and Realism

### Inertial Properties

Accurate inertial properties are crucial for realistic physics:

```xml
<link name="link_name">
  <inertial>
    <mass>1.0</mass>
    <inertia>
      <ixx>0.01</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.01</iyy>
      <iyz>0.0</iyz>
      <izz>0.01</izz>
    </inertia>
  </inertial>
</link>
```

### Surface Properties

Surface properties affect how objects interact:

- **Friction**: Determines how objects slide against each other
- **Bounce**: Controls elasticity of collisions
- **Stiffness**: Affects how rigid the contact feels

## Humanoid Robot Physics Considerations

### Balance and Stability

Humanoid robots face unique physics challenges:

1. **Center of Mass**: Must be carefully calculated and maintained
2. **Zero Moment Point (ZMP)**: Critical for stable walking
3. **Angular Momentum**: Important for dynamic movements

### Joint Limitations

Realistic joint limitations ensure physically possible movements:

```xml
<joint name="joint_name" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>
  <limit>
    <lower>-1.57</lower>  <!-- Lower limit in radians -->
    <upper>1.57</upper>   <!-- Upper limit in radians -->
    <effort>100.0</effort>  <!-- Maximum effort -->
    <velocity>1.0</velocity> <!-- Maximum velocity -->
  </limit>
</joint>
```

## Gazebo Plugins for Enhanced Physics

### Joint Control Plugins

For humanoid robots, you'll often use joint control plugins:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint1, joint2, joint3</joint_name>
  </plugin>
</gazebo>
```

### Physics Engine Selection

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Default, good for most robotics applications
- **Bullet**: Better for complex contact scenarios
- **SimBody**: Advanced for biomechanics

## Practical Example: Simple Humanoid Model Physics

Here's a simplified example of a humanoid leg with proper physics setup:

```xml
<model name="simple_leg">
  <link name="thigh">
    <inertial>
      <mass>2.0</mass>
      <inertia>
        <ixx>0.02</ixx>
        <iyy>0.02</iyy>
        <izz>0.005</izz>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyz>0.0</iyz>
      </inertia>
    </inertial>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.4</length>
        </cylinder>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.4</length>
        </cylinder>
      </geometry>
    </collision>
  </link>

  <link name="shin">
    <inertial>
      <mass>1.5</mass>
      <inertia>
        <ixx>0.01</ixx>
        <iyy>0.01</iyy>
        <izz>0.003</izz>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyz>0.0</iyz>
      </inertia>
    </inertial>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.04</radius>
          <length>0.35</length>
        </cylinder>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.04</radius>
          <length>0.35</length>
        </cylinder>
      </geometry>
    </collision>
  </link>

  <joint name="knee_joint" type="revolute">
    <parent>thigh</parent>
    <child>shin</child>
    <axis>
      <xyz>0 1 0</xyz>
      <limit>
        <lower>-2.0</lower>
        <upper>0.5</upper>
        <effort>100.0</effort>
        <velocity>2.0</velocity>
      </limit>
    </axis>
    <pose>0 -0.2 0 0 0 0</pose>
  </joint>
</model>
```

## Performance Considerations

### Simulation Step Size

- Smaller step sizes (e.g., 0.001s) provide more accurate physics but require more computation
- Larger step sizes (e.g., 0.01s) are faster but may cause instability
- Balance accuracy with performance needs

### Real-time Factor

- Real-time factor of 1.0 means simulation runs at the same speed as real time
- Values > 1.0 mean simulation runs faster than real time
- Values < 1.0 mean simulation runs slower than real time

## Troubleshooting Common Physics Issues

### Robot Falls Through Ground

- Check collision geometries are properly defined
- Verify gravity is correctly configured
- Increase contact stiffness (kp) values

### Unstable Joint Movements

- Reduce simulation step size
- Check inertial properties are realistic
- Verify joint limits are appropriate

### Excessive Jittering

- Increase damping coefficients
- Check mass properties are not too low
- Verify contact parameters are properly set

## Summary

Gazebo physics simulation provides the foundation for realistic digital twins of humanoid robots. By properly configuring gravity, collision detection, material properties, and joint limitations, you can create virtual environments that accurately reflect real-world physics. This foundation is essential for developing and testing humanoid robot control algorithms in a safe, repeatable environment.

## Next Steps

In the next chapter, we'll explore how to simulate various sensors (LiDAR, depth cameras, IMUs) that work in conjunction with these physics models to create comprehensive digital twins.