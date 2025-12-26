---
title: Chapter 3 - Human-Robot Interaction in Unity
sidebar_label: Chapter 3 - Unity Interaction
description: Creating human-robot interaction scenarios in Unity to design intuitive interfaces for controlling and monitoring simulated humanoid robots
---

# Human-Robot Interaction in Unity

## Introduction to Unity for Human-Robot Interaction

Unity is a powerful game engine that excels at creating interactive 3D environments and user interfaces. For digital twins of humanoid robots, Unity provides an intuitive platform for human-robot interaction, visualization, and monitoring that complements the physics-focused simulation capabilities of Gazebo.

### Why Unity for Human-Robot Interaction?

Unity offers several advantages for human-robot interaction:

- **Intuitive Visualization**: Realistic 3D rendering for easy understanding of robot state
- **Flexible UI/UX**: Powerful tools for creating custom interfaces
- **Real-time Interaction**: Immediate feedback for user commands
- **Cross-platform Deployment**: Interfaces that work across different devices
- **Asset Ecosystem**: Extensive library of 3D models and components

## Setting Up Unity for Robotics

### Unity Robotics Setup

To set up Unity for robotics applications, you'll typically use the Unity Robotics Hub:

1. **Unity Robotics Package**: Provides ROS integration
2. **URDF Importer**: Allows importing robot models from URDF files
3. **Robotics Simulation Tools**: Utilities for physics and sensor simulation

### Basic Scene Structure

A typical Unity scene for humanoid robot interaction includes:

```
Scene
├── Robot Model (Imported from URDF or custom)
├── UI Canvas (For controls and monitoring)
├── Main Camera (For visualization)
├── Lighting (For realistic rendering)
└── ROS Connection Manager (For communication)
```

## Creating Robot Visualization

### Importing Robot Models

For humanoid robots, you can import models in several ways:

1. **URDF Import**: Direct import from ROS URDF files
2. **FBX/GLTF**: Pre-built 3D models
3. **Procedural Generation**: Scripted creation of robot parts

Example of importing a humanoid robot:

```csharp
using UnityEngine;
using Unity.Robotics.URDFImporter;

public class RobotLoader : MonoBehaviour
{
    public string urdfPath;

    void Start()
    {
        URDFRobot robot = URDFLoader.LoadRobotAtPath(urdfPath);
        robot.transform.SetParent(transform);
        robot.transform.localPosition = Vector3.zero;
        robot.transform.localRotation = Quaternion.identity;
    }
}
```

### Robot Visualization Components

A well-designed robot visualization includes:

- **Joint Visualization**: Clear representation of joint movements
- **Sensor Visualization**: Visual indicators for sensor data
- **Status Indicators**: Color-coded feedback for robot state
- **Trajectory Visualization**: Path planning and movement prediction

## Designing Human-Robot Interfaces

### Control Interfaces

Effective control interfaces for humanoid robots should include:

#### 1. Teleoperation Controls

```csharp
using UnityEngine;

public class RobotTeleopController : MonoBehaviour
{
    public GameObject robot;
    public float moveSpeed = 1.0f;
    public float rotateSpeed = 50.0f;

    void Update()
    {
        // Movement controls
        float moveVertical = Input.GetAxis("Vertical");
        float moveHorizontal = Input.GetAxis("Horizontal");

        Vector3 movement = new Vector3(moveHorizontal, 0.0f, moveVertical);
        robot.transform.Translate(movement * moveSpeed * Time.deltaTime);

        // Rotation controls
        if (Input.GetKey(KeyCode.Q))
            robot.transform.Rotate(Vector3.up, -rotateSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.E))
            robot.transform.Rotate(Vector3.up, rotateSpeed * Time.deltaTime);
    }
}
```

#### 2. Waypoint Navigation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class WaypointNavigator : MonoBehaviour
{
    public List<Vector3> waypoints = new List<Vector3>();
    public float moveSpeed = 2.0f;
    private int currentWaypoint = 0;

    void Update()
    {
        if (waypoints.Count > 0 && currentWaypoint < waypoints.Count)
        {
            Vector3 target = waypoints[currentWaypoint];
            transform.position = Vector3.MoveTowards(transform.position, target, moveSpeed * Time.deltaTime);

            if (Vector3.Distance(transform.position, target) < 0.1f)
            {
                currentWaypoint++;
                if (currentWaypoint >= waypoints.Count)
                    currentWaypoint = 0; // Loop back to start
            }
        }
    }

    public void AddWaypoint(Vector3 position)
    {
        waypoints.Add(position);
    }
}
```

### Monitoring Interfaces

Effective monitoring interfaces display:

#### 1. Sensor Data Visualization

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class SensorMonitor : MonoBehaviour
{
    public Text lidarDataText;
    public Text imuDataText;
    public Text cameraFeedText;

    // Example data structures
    private List<float> lidarReadings = new List<float>();
    private Vector3 imuAcceleration;
    private Vector3 imuAngularVelocity;

    void Update()
    {
        // Update UI with sensor data
        UpdateLidarDisplay();
        UpdateIMUDisplay();
        UpdateCameraDisplay();
    }

    void UpdateLidarDisplay()
    {
        if (lidarDataText != null)
        {
            string lidarStr = "LiDAR: ";
            for (int i = 0; i < Mathf.Min(10, lidarReadings.Count); i++)
            {
                lidarStr += lidarReadings[i].ToString("F2") + " ";
            }
            lidarDataText.text = lidarStr;
        }
    }

    void UpdateIMUDisplay()
    {
        if (imuDataText != null)
        {
            imuDataText.text = $"IMU Acc: ({imuAcceleration.x:F2}, {imuAcceleration.y:F2}, {imuAcceleration.z:F2})\n" +
                              $"IMU Vel: ({imuAngularVelocity.x:F2}, {imuAngularVelocity.y:F2}, {imuAngularVelocity.z:F2})";
        }
    }

    void UpdateCameraDisplay()
    {
        // Update camera feed visualization
    }
}
```

#### 2. Robot Status Panel

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotStatusPanel : MonoBehaviour
{
    public Slider batteryLevelSlider;
    public Text connectionStatusText;
    public Text jointStatusText;
    public Image statusIndicator;

    public Color connectedColor = Color.green;
    public Color disconnectedColor = Color.red;

    void Update()
    {
        UpdateStatus();
    }

    void UpdateStatus()
    {
        // Update battery level
        if (batteryLevelSlider != null)
        {
            // Simulated battery level
            float batteryLevel = GetSimulatedBatteryLevel();
            batteryLevelSlider.value = batteryLevel;
        }

        // Update connection status
        if (connectionStatusText != null)
        {
            bool isConnected = IsRobotConnected();
            connectionStatusText.text = isConnected ? "CONNECTED" : "DISCONNECTED";
            statusIndicator.color = isConnected ? connectedColor : disconnectedColor;
        }

        // Update joint status
        if (jointStatusText != null)
        {
            jointStatusText.text = GetJointStatusString();
        }
    }

    float GetSimulatedBatteryLevel()
    {
        // Simulate battery level over time
        return Mathf.Clamp01(1.0f - (Time.time % 1000) / 1000.0f);
    }

    bool IsRobotConnected()
    {
        // Check connection status
        return true; // Simplified for example
    }

    string GetJointStatusString()
    {
        // Return joint status information
        return "All joints nominal";
    }
}
```

## Unity-ROS Integration

### Setting Up ROS Communication

Unity can communicate with ROS systems using the ROS TCP Connector:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter.Control;

public class UnityROSConnector : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to robot state topics
        ros.Subscribe<sensor_msgs.JointState>("/joint_states", JointStateCallback);

        // Subscribe to sensor topics
        ros.Subscribe<sensor_msgs.LaserScan>("/scan", LaserScanCallback);
        ros.Subscribe<sensor_msgs.Imu>("/imu/data", IMUCallback);
    }

    void JointStateCallback(sensor_msgs.JointState jointState)
    {
        // Process joint state data
        UpdateRobotJoints(jointState);
    }

    void LaserScanCallback(sensor_msgs.LaserScan laserScan)
    {
        // Process laser scan data
        UpdateLidarVisualization(laserScan);
    }

    void IMUCallback(sensor_msgs.Imu imuData)
    {
        // Process IMU data
        UpdateIMUVisualization(imuData);
    }

    void UpdateRobotJoints(sensor_msgs.JointState jointState)
    {
        // Update robot model based on joint states
    }

    void UpdateLidarVisualization(sensor_msgs.LaserScan laserScan)
    {
        // Update lidar visualization in Unity
    }

    void UpdateIMUVisualization(sensor_msgs.Imu imuData)
    {
        // Update IMU visualization in Unity
    }
}
```

### Publishing Commands

Sending commands from Unity to the robot:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotCommandPublisher : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        var twist = new TwistMsg();
        twist.linear = new Vector3Msg(linearX, 0, 0);
        twist.angular = new Vector3Msg(0, 0, angularZ);

        ros.Publish("/cmd_vel", twist);
    }

    public void SendJointCommands(Dictionary<string, float> jointPositions)
    {
        var jointMsg = new sensor_msgs.JointStateMsg();
        jointMsg.name = new string[jointPositions.Keys.Count];
        jointMsg.position = new double[jointPositions.Values.Count];

        int i = 0;
        foreach (var kvp in jointPositions)
        {
            jointMsg.name[i] = kvp.Key;
            jointMsg.position[i] = kvp.Value;
            i++;
        }

        ros.Publish("/joint_commands", jointMsg);
    }
}
```

## Advanced Interaction Techniques

### Gesture-Based Control

```csharp
using UnityEngine;

public class GestureController : MonoBehaviour
{
    public Camera mainCamera;
    private Vector3 lastMousePosition;
    private bool isDragging = false;

    void Update()
    {
        HandleMouseInput();
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            lastMousePosition = Input.mousePosition;
            isDragging = true;
        }
        else if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
        }
        else if (isDragging)
        {
            Vector3 currentMousePos = Input.mousePosition;
            Vector3 mouseDelta = currentMousePos - lastMousePosition;

            // Interpret mouse drag as robot movement command
            ProcessGesture(mouseDelta);

            lastMousePosition = currentMousePos;
        }
    }

    void ProcessGesture(Vector3 delta)
    {
        // Convert mouse gesture to robot command
        float linear = delta.y * 0.01f;
        float angular = -delta.x * 0.01f;

        // Send command to robot
        SendVelocityCommand(linear, angular);
    }

    void SendVelocityCommand(float linear, float angular)
    {
        // Implementation depends on your communication method
    }
}
```

### Voice Command Integration

```csharp
using UnityEngine;

public class VoiceCommandHandler : MonoBehaviour
{
    private bool listening = false;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ToggleListening();
        }
    }

    void ToggleListening()
    {
        listening = !listening;
        if (listening)
        {
            StartVoiceRecognition();
        }
        else
        {
            StopVoiceRecognition();
        }
    }

    void StartVoiceRecognition()
    {
        // Initialize voice recognition system
        Debug.Log("Listening for voice commands...");
    }

    void StopVoiceRecognition()
    {
        // Stop voice recognition system
        Debug.Log("Voice recognition stopped.");
    }

    void ProcessVoiceCommand(string command)
    {
        command = command.ToLower();

        switch (command)
        {
            case "move forward":
                SendVelocityCommand(0.5f, 0.0f);
                break;
            case "turn left":
                SendVelocityCommand(0.0f, 0.5f);
                break;
            case "turn right":
                SendVelocityCommand(0.0f, -0.5f);
                break;
            case "stop":
                SendVelocityCommand(0.0f, 0.0f);
                break;
            default:
                Debug.Log("Unknown command: " + command);
                break;
        }
    }

    void SendVelocityCommand(float linear, float angular)
    {
        // Send command to robot
    }
}
```

## VR/AR Integration for Enhanced Interaction

### VR Controller Support

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRControllerHandler : MonoBehaviour
{
    public GameObject leftController;
    public GameObject rightController;

    void Update()
    {
        UpdateVRControllers();
    }

    void UpdateVRControllers()
    {
        // Get VR controller inputs
        if (leftController != null)
        {
            // Handle left controller input
            HandleControllerInput(leftController, XRNode.LeftHand);
        }

        if (rightController != null)
        {
            // Handle right controller input
            HandleControllerInput(rightController, XRNode.RightHand);
        }
    }

    void HandleControllerInput(GameObject controller, XRNode node)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);

        if (device.isValid)
        {
            // Get controller position and rotation
            Vector3 position;
            Quaternion rotation;

            if (device.TryGetFeatureValue(CommonUsages.devicePosition, out position))
            {
                controller.transform.position = position;
            }

            if (device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation))
            {
                controller.transform.rotation = rotation;
            }

            // Handle trigger and button presses
            float triggerValue;
            if (device.TryGetFeatureValue(CommonUsages.trigger, out triggerValue) && triggerValue > 0.1f)
            {
                HandleGripCommand(triggerValue);
            }
        }
    }

    void HandleGripCommand(float gripValue)
    {
        // Map grip value to robot action
    }
}
```

## Best Practices for Unity Interfaces

### Performance Optimization

1. **LOD Systems**: Use Level of Detail to reduce rendering complexity
2. **Occlusion Culling**: Don't render objects not visible to the camera
3. **Object Pooling**: Reuse objects instead of constantly creating/destroying
4. **Efficient Updates**: Only update UI elements when data changes

### User Experience Design

1. **Consistency**: Maintain consistent interaction patterns
2. **Feedback**: Provide immediate visual/auditory feedback
3. **Accessibility**: Support various input methods and user needs
4. **Intuitiveness**: Design interfaces that are easy to understand

### Safety Considerations

1. **Command Validation**: Validate all commands before sending to robot
2. **Emergency Stop**: Always provide an easy way to stop robot motion
3. **Limit Enforcement**: Ensure commands stay within safe operational limits
4. **Status Monitoring**: Continuously monitor robot status during operation

## Practical Example: Complete Unity Interface

Here's a complete example of a Unity interface for humanoid robot interaction:

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class HumanoidRobotInterface : MonoBehaviour
{
    [Header("Robot Components")]
    public GameObject robotModel;
    public Camera mainCamera;

    [Header("UI Elements")]
    public Slider linearSpeedSlider;
    public Slider angularSpeedSlider;
    public Text statusText;
    public Button emergencyStopButton;
    public Toggle followModeToggle;

    [Header("Sensor Visualization")]
    public GameObject lidarPointCloud;
    public GameObject imuIndicator;

    private ROSConnection ros;
    private bool followMode = false;
    private float linearSpeed = 0.5f;
    private float angularSpeed = 0.5f;

    void Start()
    {
        ros = ROSConnection.instance;

        // Setup UI event listeners
        SetupUIEvents();

        // Initialize robot state
        InitializeRobot();
    }

    void SetupUIEvents()
    {
        if (linearSpeedSlider != null)
            linearSpeedSlider.onValueChanged.AddListener(OnLinearSpeedChanged);

        if (angularSpeedSlider != null)
            angularSpeedSlider.onValueChanged.AddListener(OnAngularSpeedChanged);

        if (emergencyStopButton != null)
            emergencyStopButton.onClick.AddListener(EmergencyStop);

        if (followModeToggle != null)
            followModeToggle.onValueChanged.AddListener(OnFollowModeChanged);
    }

    void OnLinearSpeedChanged(float value)
    {
        linearSpeed = value;
    }

    void OnAngularSpeedChanged(float value)
    {
        angularSpeed = value;
    }

    void OnFollowModeChanged(bool value)
    {
        followMode = value;
    }

    void Update()
    {
        HandleInput();
        UpdateCamera();
        UpdateStatus();
    }

    void HandleInput()
    {
        float moveVertical = Input.GetAxis("Vertical");
        float moveHorizontal = Input.GetAxis("Horizontal");

        float linear = moveVertical * linearSpeed;
        float angular = moveHorizontal * angularSpeed;

        SendVelocityCommand(linear, angular);
    }

    void UpdateCamera()
    {
        if (followMode && robotModel != null && mainCamera != null)
        {
            // Follow the robot with the camera
            Vector3 offset = new Vector3(0, 2, -3);
            mainCamera.transform.position = robotModel.transform.position + offset;
            mainCamera.transform.LookAt(robotModel.transform);
        }
    }

    void UpdateStatus()
    {
        if (statusText != null)
        {
            statusText.text = $"Linear: {linearSpeed:F2}, Angular: {angularSpeed:F2}\n" +
                             $"Follow Mode: {(followMode ? "ON" : "OFF")}";
        }
    }

    void SendVelocityCommand(float linear, float angular)
    {
        if (ros != null)
        {
            var twist = new RosMessageTypes.Geometry.TwistMsg();
            twist.linear = new RosMessageTypes.Geometry.Vector3Msg(linear, 0, 0);
            twist.angular = new RosMessageTypes.Geometry.Vector3Msg(0, 0, angular);

            ros.Publish("/cmd_vel", twist);
        }
    }

    void EmergencyStop()
    {
        SendVelocityCommand(0, 0);
        Debug.Log("Emergency stop activated!");
    }

    void InitializeRobot()
    {
        // Initialize robot to known state
        SendVelocityCommand(0, 0);
    }
}
```

## Troubleshooting Common Unity Integration Issues

### Connection Problems

- Verify ROS TCP connector is properly configured
- Check IP addresses and ports are correct
- Ensure firewall allows the connection

### Performance Issues

- Reduce polygon count of robot models
- Use occlusion culling for large environments
- Limit update frequency of sensor visualizations

### Synchronization Problems

- Ensure Unity and ROS clocks are properly synchronized
- Use appropriate buffer sizes for data transmission
- Implement proper error handling for dropped messages

## Summary

Unity provides a powerful platform for creating intuitive human-robot interaction interfaces for digital twin applications. By combining Unity's visualization capabilities with ROS communication, you can create comprehensive interfaces for monitoring and controlling simulated humanoid robots. Proper interface design, performance optimization, and safety considerations ensure effective and safe human-robot interaction.

## Next Steps

With the complete digital twin system now implemented using Gazebo for physics simulation, sensor simulation, and Unity for human-robot interaction, you have a comprehensive foundation for developing and testing humanoid robot applications in a safe, controlled virtual environment.