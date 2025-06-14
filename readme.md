# Robot Arm Wave Demo - ROS 2 Visualization

This demo creates a 6-DOF robot arm that waves in RViz using ROS 2.

## Directory Structure

Create a new directory for your project and place all these files in it:

```
robot_arm_wave_demo/
├── simple_arm.urdf          # Robot description file
├── robot_arm_wave.py        # Wave animation controller
├── launch_arm_demo.py       # Launch file
├── arm_demo.rviz           # RViz configuration
└── README.md               # This file
```

## Prerequisites

- ROS 2 (Humble, Iron, or Jazzy)
- Python 3
- The following ROS 2 packages:
  ```bash
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-robot-state-publisher
  sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
  sudo apt install ros-$ROS_DISTRO-rviz2
  ```

## Setup Instructions

1. **Create a new directory and navigate to it:**
   ```bash
   mkdir -p ~/robot_arm_wave_demo
   cd ~/robot_arm_wave_demo
   ```

2. **Save all the provided files in this directory**

3. **Make the Python files executable:**
   ```bash
   chmod +x robot_arm_wave.py
   chmod +x launch_arm_demo.py
   ```

4. **Source your ROS 2 installation:**
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```

## Running the Demo

### Method 1: Using the Launch File (Recommended)
```bash
ros2 launch ./launch_arm_demo.py
```

This will start:
- Robot State Publisher (publishes robot model to TF)
- Wave animation node
- RViz with pre-configured view

### Method 2: Running Components Individually

If you prefer to run each component separately:

**Terminal 1 - Robot State Publisher:**
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_arm.urdf)"
```

**Terminal 2 - Wave Animation:**
```bash
python3 robot_arm_wave.py
```

**Terminal 3 - RViz:**
```bash
rviz2 -d arm_demo.rviz
```

## What You Should See

- A colorful 6-DOF robot arm in RViz
- The arm's "wrist" joint waving back and forth
- The arm positioned in a greeting pose with shoulder raised and elbow bent
- Smooth animation at 30 Hz

## Customizing the Wave Motion

Edit `robot_arm_wave.py` to modify:
- `wave_freq`: Wave speed (default: 1.5 Hz)
- `wave_amplitude`: How far the wrist moves (default: 0.8 radians)
- Joint positions: Modify the base pose or add motion to other joints

## Troubleshooting

1. **"No module named 'rclpy'" error:**
   - Make sure you've sourced ROS 2: `source /opt/ros/$ROS_DISTRO/setup.bash`

2. **Robot not visible in RViz:**
   - Check that Fixed Frame is set to "base_link"
   - Ensure RobotModel display is enabled
   - Verify robot_description topic is being published

3. **Launch file fails:**
   - Make sure all files are in the same directory
   - Check that Python files are executable
   - Try running components individually to isolate issues

4. **Permission denied errors:**
   - Run: `chmod +x *.py`

## Understanding the Components

- **URDF File**: Defines the robot's physical structure (links and joints)
- **Wave Controller**: Publishes joint states to animate the robot
- **Launch File**: Starts all necessary nodes together
- **RViz Config**: Pre-configured 3D visualization settings

## Next Steps

- Modify the URDF to change robot appearance
- Add more complex motions (e.g., figure-8 patterns, multi-joint coordination)
- Implement keyboard control for interactive movement
- Add grippers or tools to the end effector
- Create different gestures (thumbs up, peace sign, etc.)