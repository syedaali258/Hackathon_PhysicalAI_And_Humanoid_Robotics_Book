---
title: Chapter 3 - Humanoid Modeling with URDF
sidebar_label: Chapter 3 - URDF Modeling
description: Understanding and modifying URDF files for humanoid robot modeling
---

# Humanoid Modeling with URDF

## Introduction

In this chapter, we'll explore Universal Robot Description Format (URDF), the standard XML format used in ROS for representing robots. URDF allows us to describe robot models with their links, joints, and other properties, which is essential for simulation, visualization, and control of humanoid robots.

## What is URDF?

URDF (Universal Robot Description Format) is an XML format used in ROS to describe robot models. It contains information about:
- Robot's physical structure (links and joints)
- Kinematic and dynamic properties
- Visual and collision properties
- Inertial properties

URDF is fundamental to ROS robotics as it enables:
- Robot simulation in tools like Gazebo
- Robot visualization in RViz
- Kinematic analysis and planning
- Robot control algorithms

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Elements Explained

### Links

Links represent rigid parts of the robot. Each link has:
- **Visual**: How the link looks in visualization
- **Collision**: How the link interacts in collision detection
- **Inertial**: Physical properties for dynamics simulation

### Joints

Joints connect links together. Common joint types:
- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement between links
- **floating**: 6 DOF movement (rarely used)

### Materials and Colors

You can define materials for visualization:

```xml
<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0"/>
</material>

<link name="base_link">
  <visual>
    <geometry>
      <box size="1.0 1.0 1.0"/>
    </geometry>
    <material name="blue"/>
  </visual>
</link>
```

## URDF for Humanoid Robots

Humanoid robots have a more complex structure with multiple limbs. Here's a simplified example:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.05 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_hip_to_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.015"/>
    </inertial>
  </link>
</robot>
```

## Working with URDF in ROS

### Loading URDF in ROS

To use a URDF in ROS, you typically load it into the parameter server:

```bash
ros2 param set /robot_state_publisher robot_description --string-type "$(cat path/to/robot.urdf)"
```

Or in a launch file:

```xml
<launch>
  <param name="robot_description" command="xacro $(find-pkg-share my_robot_description)/urdf/robot.urdf.xacro"/>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher"/>
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher"/>
</launch>
```

### Using Xacro for Complex URDFs

Xacro is a macro language for XML that allows you to create more maintainable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.5" />
  <xacro:property name="torso_radius" value="0.15" />

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="name parent xyz rpy">
    <joint name="${parent}_to_${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Use the macro to create limbs -->
  <xacro:limb name="right_arm" parent="torso" xyz="0 0.2 0" rpy="0 0 0"/>
  <xacro:limb name="left_arm" parent="torso" xyz="0 -0.2 0" rpy="0 0 0"/>

</robot>
```

## URDF Validation and Tools

### Validating URDF

You can validate your URDF using ROS tools:

```bash
# Check for syntax errors
check_urdf path/to/robot.urdf

# Visualize the kinematic chain
urdf_to_graphiz path/to/robot.urdf
```

### Visualization

To visualize your URDF in RViz:

1. Launch robot_state_publisher
2. Add RobotModel display in RViz
3. Set the robot description parameter

## Best Practices for URDF

1. **Use consistent naming**: Use descriptive names for links and joints
2. **Organize complex models**: Use Xacro macros for repetitive structures
3. **Set proper inertial properties**: Critical for physics simulation
4. **Consider collision vs visual**: Collision models can be simpler than visual models
5. **Use fixed joints for static attachments**: Don't overcomplicate the kinematic tree

## URDF in Simulation

URDF is essential for robot simulation in Gazebo and other simulators:

```xml
<!-- Add Gazebo-specific plugins -->
<gazebo reference="left_wheel">
  <mu1 value="1.0"/>
  <mu2 value="1.0"/>
  <kp value="10000000.0"/>
  <kd value="1.0"/>
</gazebo>

<!-- Add transmission for control -->
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Summary

In this chapter, we've covered:

- The fundamentals of URDF and its role in ROS
- Basic URDF structure with links and joints
- Creating humanoid robot models
- Using Xacro for complex models
- Validation and visualization tools
- Best practices for URDF development

URDF is a critical skill for robotics development, especially for humanoid robots where the kinematic structure is complex and requires careful modeling.

## Next Steps

With the fundamentals of ROS 2, Python agents, and URDF modeling covered, you now have a solid foundation for developing sophisticated robotic systems. The next modules will build on these concepts to explore simulation environments, advanced platforms, and vision-language-action models.