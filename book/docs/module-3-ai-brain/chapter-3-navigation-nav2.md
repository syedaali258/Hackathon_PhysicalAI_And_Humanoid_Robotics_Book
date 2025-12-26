---
title: Chapter 3 - Navigation and Path Planning with Nav2
sidebar_label: Chapter 3 - Navigation with Nav2
description: Understanding and implementing navigation and path planning with the Nav2 stack for humanoid robot autonomy
---

# Navigation and Path Planning with Nav2

## Introduction to Navigation and Path Planning

Navigation is a critical capability for humanoid robots, enabling them to autonomously move through environments to reach desired goals while avoiding obstacles. The Navigation2 (Nav2) stack is the standard ROS 2 framework for mobile robot navigation, providing a comprehensive set of tools for path planning, execution, and recovery.

### Why Navigation Matters for Humanoid Robots

Humanoid robots have unique navigation requirements compared to wheeled robots:

- **Human-like Movement**: Must navigate in human environments with stairs, narrow passages, and social considerations
- **Dynamic Obstacles**: Need to handle moving humans and other dynamic obstacles gracefully
- **Social Navigation**: Must respect human social spaces and navigation patterns
- **Complex Terrain**: Need to handle uneven terrain and obstacles that require complex movement patterns

### Nav2 Architecture Overview

The Nav2 stack is built around a behavior tree architecture that allows for complex navigation behaviors:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Navigation    │    │  Behavior Tree   │    │   Controllers   │
│   Goal          │───▶│    Executor      │───▶│   & Executors   │
│                 │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Map Server    │    │   Path Planner   │    │  Local Planner  │
│ (Static/Global) │◀───┤   (Global)       │◀───┤   (Local)       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Costmap       │    │   Path Follow   │    │   Collision     │
│  Management     │───▶│    & Control    │───▶│   Avoidance     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Nav2 Core Components

### 1. Map Server

The map server provides static and local costmaps for navigation:

```python
# Example Nav2 map server configuration
map_server_params = {
    'yaml_filename': '/path/to/map.yaml',
    'topic_name': 'map',
    'frame_id': 'map',
    'output': 'screen',
    'use_sim_time': True,  # Important for simulation environments
    'publish_frequency': 1.0,
    'save_map_timeout': 5.0
}
```

### 2. Global Planner

The global planner computes the initial path from start to goal:

```python
# Global planner configuration
global_planner_config = {
    'name': 'GridBased',
    'plugin': 'nav2_navfn_planner/NavfnPlanner',
    'tolerance': 0.5,  # Acceptable distance to goal
    'use_astar': False,
    'allow_unknown': True,
    'planner_thread_timeout': 100  # ms
}
```

### 3. Local Planner

The local planner handles path following and obstacle avoidance:

```python
# Local planner configuration for humanoid robots
local_planner_config = {
    'name': 'TBPLocalPlanner',
    'plugin': 'nav2_mppi_controller/MppiController',
    'frequency': 20.0,  # Hz
    'min_vel_x': 0.0,
    'max_vel_x': 0.5,   # Slower for humanoid stability
    'max_vel_theta': 1.0,
    'min_speed_xy': 0.1,
    'max_speed_xy': 0.5,
    'min_speed_theta': 0.0,
    'acc_lim_x': 2.5,
    'acc_lim_theta': 3.2,
    'decel_lim_x': -2.5,
    'decel_lim_theta': -3.2,
    'vx_samples': 20,
    'vy_samples': 5,
    'vtheta_samples': 20,
    'sim_time': 1.7,
    'time_step': 0.1,
    'penalty_epsilon': 0.05,
    'ctrl_freq': 20.0,
    'rolling_window_width': 3.0,
    'rolling_window_height': 3.0,
    'collision_cost_threshold': 1000,
    'cost_scaling_dist': 0.6,
    'inflation_cost_scaling_factor': 3.0
}
```

### 4. Behavior Trees

Behavior trees define complex navigation behaviors:

```xml
<!-- Example Nav2 behavior tree -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <RecoveryNode number_of_retries="6">
                    <SequenceStar name="ComputeAndExecutePath">
                        <GoalReached/>
                        <ComputePathToPose goal="{goal}" path="{path}"/>
                        <SmoothPath path="{path}" smooth_path="{smooth_path}"/>
                        <FollowPath path="{smooth_path}" velocity="{velocity}"/>
                    </SequenceStar>
                    <ReactiveFallback name="MoveBaseFallback">
                        <GoalUpdated/>
                        <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
                        <RecoveryNode number_of_retries="2">
                            <SequenceStar name="ClearingActions">
                                <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
                                <Spin spin_dist="1.57"/>
                            </SequenceStar>
                            <BackUp backup_dist="0.15" backup_speed="0.05"/>
                        </RecoveryNode>
                    </ReactiveFallback>
                </RecoveryNode>
            </RateController>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Isaac Sim Integration with Nav2

### Simulation Environment Setup

Setting up a navigation environment in Isaac Sim involves creating a world with appropriate semantics for Nav2:

```python
# Isaac Sim Nav2 environment setup
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import set_targets
import carb

class HumanoidNavigationWorld(World):
    def __init__(self):
        super().__init__(stage_units_in_meters=1.0)

        # Add humanoid robot with navigation sensors
        self._add_humanoid_robot()

        # Add navigation sensors (LIDAR, cameras, IMU)
        self._add_navigation_sensors()

        # Create semantically tagged navigation environment
        self._add_semantic_navigation_environment()

    def _add_humanoid_robot(self):
        """Add humanoid robot suitable for navigation tasks"""
        # Add a humanoid robot with appropriate navigation configuration
        add_reference_to_stage(
            usd_path="/Isaac/Robots/NVIDIA/Isaac/Robot_ARM/urdf/nv_humannoid/nv_humanoid_instanceable.usd",
            prim_path="/World/HumanoidRobot"
        )

    def _add_navigation_sensors(self):
        """Add sensors required for Nav2 navigation"""
        # Add LIDAR for obstacle detection
        from omni.isaac.sensor import RotatingLidarSensor

        self.lidar = RotatingLidarSensor(
            prim_path="/World/HumanoidRobot/base_scan",
            translation=np.array([0.0, 0.0, 0.5]),  # Mount on torso
            name="navigation_lidar",
            rotation_frequency=20,  # Hz
            samples_per_scan=360,
            horizontal_samples=360,
            vertical_samples=1,
            horizontal_fov=360,
            vertical_fov=10,
            max_range=10.0,
            min_range=0.1
        )

        # Add front-facing camera for visual navigation
        from omni.isaac.sensor import Camera

        self.front_camera = Camera(
            prim_path="/World/HumanoidRobot/front_camera",
            position=np.array([0.2, 0.0, 0.8]),  # Head-level position
            frequency=30,
            resolution=(640, 480)
        )

    def _add_semantic_navigation_environment(self):
        """Add environment with semantic annotations for navigation"""
        # Add a structured indoor environment
        add_reference_to_stage(
            usd_path="/Isaac/Environments/Simple_Warehouse/simple_warehouse.usd",
            prim_path="/World/NavigationEnvironment"
        )

        # Add semantic annotations to objects
        from omni.isaac.core.utils.semantics import add_semantics

        # Add semantic labels to navigation-relevant objects
        # This allows Nav2 to understand object types for navigation planning
        # For example, marking doorways, obstacles, and navigable areas
        # This would be done with Isaac Sim's semantic labeling tools
```

### Nav2 Configuration for Isaac Sim

Here's how to configure Nav2 for use with Isaac Sim environments:

```yaml
# nav2_config_isaac.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a_distance: 0.2
    update_min_d_angle: 0.5
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_are_equal_poses_condition_bt_node
    - nav2_are_amcl_consistent_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_velocity_check_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB parameters
    FollowPath:
      plugin: "nav2_mppi_controller/MppiController"
      frequency: 20.0
      min_vel_x: 0.0
      max_vel_x: 0.5  # Slower for humanoid stability
      max_vel_theta: 1.0
      min_speed_xy: 0.1
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      time_step: 0.1
      penalty_epsilon: 0.05
      ctrl_freq: 20.0
      rolling_window_width: 3.0
      rolling_window_height: 3.0
      collision_cost_threshold: 1000
      cost_scaling_dist: 0.6
      inflation_cost_scaling_factor: 3.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Humanoid-specific radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /local_plan/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # Humanoid-specific radius
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /global_plan/scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.8

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: False
      allow_unknown: True
      planner_thread_timeout: 100
```

## Path Planning Algorithms for Humanoid Robots

### Humanoid-Specific Navigation Considerations

Humanoid robots require special consideration in path planning:

#### 1. Kinematic Constraints

Humanoid robots have different kinematic constraints than wheeled robots:

```python
# Humanoid kinematic constraints for navigation
humanoid_constraints = {
    # Step size limitations
    'max_step_size': 0.3,  # Maximum distance per step

    # Turning radius based on bipedal mechanics
    'min_turning_radius': 0.4,  # Minimum turning radius for stable walking

    # Balance constraints
    'max_incline_angle': 15.0,  # Maximum incline angle in degrees
    'max_surface_roughness': 0.05,  # Maximum surface roughness in meters

    # Social navigation
    'personal_space_radius': 1.0,  # Radius to maintain from humans
    'social_lane_width': 1.2,  # Minimum width for comfortable human passage
}
```

#### 2. Dynamic Obstacle Handling

Humanoid robots need to handle dynamic obstacles differently:

```python
# Dynamic obstacle handling for human-aware navigation
dynamic_obstacle_handling = {
    'prediction_horizon': 3.0,  # Seconds to predict human movement
    'social_comfort_zone': 1.0,  # Distance to maintain from humans
    'motion_prediction_model': 'constant_velocity',  # Human motion prediction
    'avoidance_strategy': 'social_force_model',  # Strategy for human-aware navigation
    'interaction_threshold': 2.0,  # Distance to trigger social interaction
    'waiting_behavior': True,  # Whether to wait for humans to pass
}
```

### Navigation Recovery Behaviors

Nav2 includes recovery behaviors for handling navigation failures:

```python
# Recovery behavior configuration for humanoid robots
recovery_behaviors = [
    {
        'name': 'spin',
        'type': 'nav2_recoveries/Spin'
    },
    {
        'name': 'backup',
        'type': 'nav2_recoveries/BackUp'
    },
    {
        'name': 'drive_on_heading',
        'type': 'nav2_recoveries/DriveOnHeading'
    },
    {
        'name': 'assisted_teleop',
        'type': 'nav2_recoveries/AssistedTeleop'
    },
    {
        'name': 'wait',
        'type': 'nav2_recoveries/Wait',
        'condition': 'not_moving'
    }
]

# Humanoid-specific recovery parameters
humanoid_recovery_params = {
    'spin': {
        'min_duration': 1.0,
        'max_duration': 10.0,
        'rate': 0.5  # Slower spin for stability
    },
    'backup': {
        'backup_dist': 0.2,  # Shorter backup distance for humanoid stability
        'backup_speed': 0.1,
        'translate_only': True  # Only translate, no rotation during backup
    }
}
```

## Practical Implementation Example

### Isaac Sim Navigation Launch

Here's how to launch navigation in Isaac Sim with Nav2:

```python
# Isaac Sim Navigation Launch Example
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare("isaac_sim_navigation"), "config", "nav2_config_isaac.yaml"]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=PathJoinSubstitution([FindPackageShare("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"]),
        description='Full path to the behavior tree xml file to use')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='false',
        description='Whether to set the map subscriber to transient local')

    # Nodes
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'diff_drive_controller/cmd_vel_unstamped')])

    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'diff_drive_controller/cmd_vel_unstamped'),
                    ('odom', 'diff_drive_controller/odom')])

    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    recoveries_server_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'recoveries_server',
                                    'bt_navigator']}])

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)

    # Add nodes
    ld.add_action(bt_navigator_cmd)
    ld.add_action(controller_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(recoveries_server_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
```

### Isaac ROS Navigation Integration

Here's how to integrate Isaac Sim with Isaac ROS navigation components:

```python
# Isaac ROS Navigation Integration Example
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class IsaacNavigationManager(Node):
    def __init__(self):
        super().__init__('isaac_navigation_manager')

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create publisher for sending goals
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Initialize Isaac-specific navigation parameters
        self.setup_isaac_navigation()

    def setup_isaac_navigation(self):
        """Configure navigation parameters for Isaac Sim environment"""
        # Set Isaac-specific parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('isaac.navigation.max_velocity', 0.3),  # Humanoid-specific speed
                ('isaac.navigation.min_distance_to_obstacle', 0.5),
                ('isaac.navigation.social_comfort_zone', 1.0),
                ('isaac.navigation.recovery_attempts', 3)
            ]
        )

    def send_navigation_goal(self, x, y, z=0.0, quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0):
        """Send a navigation goal to Nav2 via Isaac Sim"""
        # Wait for action server
        self.nav_client.wait_for_server()

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z

        goal_msg.pose.pose.orientation.w = quat_w
        goal_msg.pose.pose.orientation.x = quat_x
        goal_msg.pose.pose.orientation.y = quat_y
        goal_msg.pose.pose.orientation.z = quat_z

        # Send goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        return future

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        # Check if navigation was successful
        if result:
            self.get_logger().info('Navigation completed successfully!')
        else:
            self.get_logger().info('Navigation failed!')

def main(args=None):
    rclpy.init(args=args)

    navigation_manager = IsaacNavigationManager()

    # Example: Send navigation goal to Isaac Sim environment
    # This would typically be called based on user input or planning algorithm
    navigation_manager.send_navigation_goal(5.0, 3.0)  # Navigate to position (5, 3)

    try:
        rclpy.spin(navigation_manager)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Navigation Performance Considerations

### Humanoid Navigation Performance Metrics

For humanoid robots, navigation performance is measured differently than for wheeled robots:

- **Stability**: Measure of how consistently the robot maintains balance during navigation
- **Social Compliance**: How well the robot follows social navigation norms
- **Energy Efficiency**: Power consumption during navigation (important for humanoid batteries)
- **Path Smoothness**: How smoothly the robot follows the planned path
- **Human Interaction Quality**: How naturally the robot interacts with humans during navigation

### Optimization Strategies

Several strategies can optimize navigation for humanoid robots:

1. **Predictive Path Planning**: Anticipate human movements and adjust path accordingly
2. **Stability-Conscious Planning**: Consider robot stability during path generation
3. **Social Navigation**: Implement human-aware navigation behaviors
4. **Multi-Modal Path Planning**: Combine different locomotion modes (walking, stepping, etc.)

## Troubleshooting Common Issues

### Navigation Failure in Isaac Sim

Common causes and solutions:

- **Localization Lost**: Ensure proper map initialization and sensor calibration
- **Path Planning Fails**: Check map quality and obstacle detection
- **Oscillation**: Adjust controller parameters for humanoid-specific dynamics
- **Recovery Loops**: Fine-tune recovery behaviors for humanoid capabilities

### Performance Optimization

- **Costmap Resolution**: Balance accuracy with computational performance
- **Controller Frequency**: Adjust for humanoid stability requirements
- **Path Smoothing**: Apply smoothing algorithms for natural humanoid movement
- **Sensor Fusion**: Optimize sensor data integration for better obstacle detection

## Summary

Navigation and path planning with Nav2 provides humanoid robots with the capability to autonomously move through complex environments while respecting human social norms and maintaining stability. The integration with Isaac Sim enables safe testing of navigation algorithms in realistic environments before deployment on physical robots.

The Nav2 stack's modular architecture and behavior tree approach allows for complex navigation behaviors that can be customized for humanoid-specific requirements, including kinematic constraints, social navigation, and recovery behaviors appropriate for bipedal locomotion.

In the next module, we'll explore how to integrate Vision-Language-Action models that can interpret natural language commands and execute complex manipulation tasks using the perception and navigation capabilities developed in this module.