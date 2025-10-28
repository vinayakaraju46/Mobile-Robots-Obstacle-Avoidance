# ScanAnalyser — Obstacle Detection and Avoidance using LaserScan (ROS2)

This ROS2 node implements a reactive obstacle detection and avoidance system using `sensor_msgs/LaserScan` data.  
The robot adjusts its velocity and steering direction based on the proximity and position of obstacles detected in its field of view.

---

## Overview

The `ScanAnalyser` node subscribes to a **LaserScan topic** (`/scan`) and publishes velocity commands to the **cmd_vel topic** (`/cmd_vel`).  
It continuously analyzes the incoming LiDAR scan data to identify the closest obstacle and adjusts the robot’s linear and angular velocities to safely navigate around it.

The system uses two state machines:
- **Velocity Control State Machine** – governs the robot’s linear motion (speed and stop behavior).
- **Turning Control State Machine** – determines the turning direction based on obstacle position.

---

## Node Summary

| Topic | Type | Role |
|--------|------|------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Input data for obstacle detection |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command output |
| `/odom` | `nav_msgs/msg/Odometry` *(optional)* | Used for pose estimation (currently not influencing control logic) |

---

## Core Classes

### 1. `PolarScanInfo`

A simple class that stores polar coordinates of a detected obstacle:

```cpp
float d;      // Distance to obstacle
float theta;  // Angle of obstacle
```

### 2. obstacleRangeOfDetection

Defines angular regions (in radians) to categorize obstacle positions:

- Left region  
- Right region  
- Mid zones for reference  

Constructed using an angular threshold (e.g., 45°) to define the robot’s left and right detection sectors.

---

### 3. ScanAnalyser

This is the main ROS2 node that:

- Subscribes to `/scan`
- Publishes velocity commands to `/cmd_vel`
- Implements the obstacle avoidance state machines

---

## State Machines

### A. Robot Velocity State Machine (`STATES`)

This governs how fast the robot moves based on the distance to the nearest obstacle (`scaled_r`).

| State | Code | Condition | Behavior |
|--------|------|------------|-----------|
| `UNKNOWN` | 0 | Initial or no scan yet | Stop robot (no command) |
| `FULL_SPEED` | 1 | `scaled_r > rsafe` | Move at full speed |
| `SLOW_DOWN` | 2 | `rstop < scaled_r <= rsafe` | Proportionally reduce speed |
| `STOP` | 3 | `scaled_r <= rstop` | Stop the robot completely |

---

### B. Turning State Machine (TURNING_STATES)

This controls the direction of rotation when an obstacle is detected within a certain angular region.

| State | Code | Condition | Behavior |
|--------|------|------------|-----------|
| **NOT_KNOWN** | 0 | Initial state | No turning |
| **GO_STRAIGHT** | 1 | No obstacle nearby | Move straight |
| **TURN_LEFT** | 2 | Obstacle detected on right side | Turn left |
| **TURN_RIGHT** | 3 | Obstacle detected on left side | Turn right | 

---

## Working Principle

### Step 1 – Laser Scan Processing
- The node iterates over all laser readings in `/scan`.
- Invalid readings (inf, NaN, or out-of-range) are ignored.
- The closest valid obstacle distance (`rmin`) and corresponding angle (`phimin`) are identified.

### Step 2 – Scaled Distance Calculation
```
scaled_r = rmin * (1 - β * cos(phimin))
```

The parameter **β (default = 0.55)** weights the obstacle’s angular position — obstacles directly in front are treated as more critical than those to the sides.

### Step 3 – State Transitions
Depending on `scaled_r`, the robot transitions between **FULL_SPEED**, **SLOW_DOWN**, and **STOP**.

### Step 4 – Velocity Command Publishing
In each control cycle:

- The **linear velocity** is determined by the velocity state.
- The **angular velocity** is determined by the turning state.
- The command is then published to `/cmd_vel` as a `geometry_msgs/Twist` message.

---

## Key Parameters

| Parameter | Meaning | Default Value |
|------------|----------|----------------|
| `rstop` | Distance below which robot stops | 0.20 m |
| `rsafe` | Safe distance to move at full speed | 0.30 m |
| `rturn` | Threshold for beginning to turn | 0.25 m |
| `vmax` | Maximum forward velocity | 0.07 m/s |
| `vreverse` | Reverse velocity (unused yet) | -0.07 m/s |
| `angular_speed_max` | Maximum rotational speed | 0.2 rad/s |
| `β` | Angular weighting factor | 0.55 |

---

## Function Breakdown

### `scan_callback(const sensor_msgs::msg::LaserScan&)`
- Processes incoming LiDAR data.
- Updates the minimum distance and corresponding angle.
- Determines the current velocity and turning states.

### `initiateObstacleAvoidance()`
- Executes periodically via a ROS timer.
- Selects the correct motion behavior (**FULL_SPEED**, **SLOW_DOWN**, **STOP**).
- Calls `handleRobotVelocity()` with the computed linear speed.

### `handleRobotVelocity(float linear_velocity)`
- Combines linear velocity with angular control.
- Sets angular direction based on turning state (**LEFT**, **RIGHT**, **STRAIGHT**).
- Publishes `geometry_msgs/Twist` to `/cmd_vel`.

---

## Example Output

```
[INFO] [scan_analyzer]: Minimum scaled distance obstacle: rmin=0.265 m, phimin=45.000 degree
Obstacle on Left
STOP
Turning state -> 3
```


**Explanation:**
- An obstacle was detected at **0.265 m** on the **left**.
- The robot decided to **stop**.
- Turning state = **TURN_RIGHT (3)**.

---

## How to Run

```bash
# Build the package
colcon build --packages-select <your_package_name>

# Source your workspace
source install/setup.bash

# Run the node
ros2 run <your_package_name> scan_analyzer

```

## Ensure you have:

- A LiDAR sensor publishing to /scan
- A robot controller subscribed to /cmd_vel

## Future Improvements
- Integrate odometry feedback for smoother turning.
- Add dynamic parameter tuning using rclcpp::Parameter.
- Implement recovery behaviors when blocked for extended durations.
- Add visualization markers in RViz for debugging.

