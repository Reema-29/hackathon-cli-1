# URDF Examples

This directory contains URDF (Unified Robot Description Format) examples that demonstrate how to describe robot models in ROS 2.

## Overview

URDF is an XML format for representing robot models and their properties. It's used to describe the physical structure of a robot including links, joints, visual appearance, and physical properties.

## Files

- `simple_robot.urdf`: A basic wheeled robot model with a base and four wheels
- `README.md`: This file

## Understanding the Simple Robot URDF

The `simple_robot.urdf` file defines a basic wheeled robot with:

### Links
- **base_link**: The main body of the robot (rectangular box)
- **sensor_link**: A sensor mounted on top of the base (cylindrical shape)
- **front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel**: Four wheels for locomotion

### Joints
- **sensor_joint**: A fixed joint connecting the sensor to the base
- **wheel joints**: Continuous joints allowing the wheels to rotate for locomotion

### Properties Defined
- **Visual**: How the robot appears (geometry and colors)
- **Collision**: How the robot interacts with its environment
- **Inertial**: Mass and inertial properties for physics simulation

## How to Use

### 1. Validate the URDF
```bash
# Check if the URDF is valid XML and follows URDF specifications
check_urdf simple_robot.urdf
```

### 2. Visualize the Robot
```bash
# Launch RViz with the robot model
ros2 run rviz2 rviz2
# In RViz, add a RobotModel display and set the Robot Description to 'robot_description'
# Then load the URDF file as a parameter
```

### 3. Launch with Robot State Publisher
```bash
# Publish the robot model to ROS 2
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_robot.urdf)
```

## Key URDF Concepts

### Links
- Represent rigid body elements
- Have visual, collision, and inertial properties
- Connected by joints

### Joints
- Define how links move relative to each other
- Types: fixed, revolute, continuous, prismatic, etc.
- Have parent and child links

### Materials
- Define visual appearance (color, texture)
- Referenced by links for visualization

## Common Joint Types

- **Fixed**: No movement, rigid connection
- **Continuous**: Rotational joint without limits (like a wheel)
- **Revolute**: Rotational joint with limits (like an elbow)
- **Prismatic**: Linear sliding joint (like a piston)

## Best Practices

1. **Start Simple**: Begin with basic shapes and add complexity gradually
2. **Use Proper Units**: Distances in meters, masses in kilograms
3. **Check Inertias**: Proper inertial values are important for simulation
4. **Validate Regularly**: Use `check_urdf` to verify your URDF is valid
5. **Origin Transformations**: Use origin tags to position joints correctly

## Troubleshooting

### Common Issues

1. **Invalid XML**: Check for proper XML syntax and matching tags
2. **Missing Properties**: Ensure all required fields are present
3. **Joint Issues**: Verify parent/child relationships are correct
4. **Visualization Problems**: Check that geometry definitions are valid

### Useful Commands

```bash
# Check URDF validity
check_urdf simple_robot.urdf

# Print URDF to console
ros2 run xacro xacro simple_robot.urdf  # if using xacro

# View the URDF in a browser
urdf_to_graphiz simple_robot.urdf
```

## Next Steps

After understanding this basic example, try:
1. Adding more complex geometries
2. Creating articulated joints for arms or other mechanisms
3. Adding Gazebo-specific extensions for simulation
4. Using Xacro to create parameterized robot models
5. Integrating with robot state publisher for visualization in RViz