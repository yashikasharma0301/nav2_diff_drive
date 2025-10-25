# NAV2 Stack Configuration for Custom Differential Drive Robot

This project demonstrates the configuration and implementation of the ROS 2 Navigation stack (Nav2) for a custom differential drive robot for simple. The package includes SLAM-based mapping, autonomous navigation, and path planning capabilities in a simulated Gazebo environment. View the project demonstration [here](https://youtu.be/0q8tDs_0iFg).

<img width="1891" height="1083" alt="Screenshot from 2025-10-25 12-20-12" src="https://github.com/user-attachments/assets/cfaab427-540e-43d9-ba5e-b70d98cf8f59" />

## System Requirements
- **Ubuntu**: 22.04 LTS
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic

## Usage

### Installation & Setup
1. **Create Workspace and Clone Repository**
```bash
mkdir -p my_ws/src && cd my_ws/src
git clone https://github.com/yashikasharma0301/nav2_diff_drive.git
```

2. **Copy Package Contents**
```bash
cp -r nav2_diff_drive/* .
rm -rf nav2_diff_drive
```

3. **Build the Workspace**
```bash
cd ~/my_ws
colcon build
```

4. **Source the Workspace**
```bash
source install/setup.bash
```

### Running the Navigation Stack

#### Step 1: Launch Robot in Gazebo and RViz
1. **Launch the Robot in Gazebo**
```bash
ros2 launch robot_description display.launch.py
```

2. **Open RViz2 for Visualization**

In another terminal (side by side):
```bash
rviz2
```

#### Step 2: Choose Your Navigation Approach

You can proceed with navigation in two ways:

---

### Method 1: Simultaneous SLAM and Navigation

This method runs SLAM and NAV2 concurrently, allowing the robot to map the environment while navigating.

1. **Launch SLAM**

In a new terminal:
```bash
source ~/my_ws/install/setup.bash
ros2 launch slam mapping.launch.py
```

2. **Launch NAV2 Stack**

In another terminal:
```bash
source ~/my_ws/install/setup.bash
ros2 launch navigation nav2.launch.py
```

3. **Navigate the Robot**
- In RViz2, use the **2D Goal Pose** tool to set a goal position
- The robot will autonomously plan and execute a path to the goal while simultaneously building the map

---

### Method 2: Map First, Then Navigate

This method creates and saves a map first, then uses it for localization-based navigation.

#### Part A: Create and Save the Map

1. **Launch SLAM**

In a new terminal:
```bash
source ~/my_ws/install/setup.bash
ros2 launch slam mapping.launch.py
```

2. **Explore the Environment**
- Manually drive the robot around the environment to create a complete map
- Monitor the map building progress in RViz2

3. **Save the Map**

Once mapping is complete, open a new terminal and navigate to the maps directory:
```bash
cd ~/my_ws/src/slam/maps/
ros2 run nav2_map_server map_saver_cli -f nav_map
```
Press Enter to save the map. This will create two files: `nav_map.pgm` and `nav_map.yaml`

#### Part B: Configure and Launch Navigation

1. **Update Navigation Parameters**

- Open the navigation parameters file and uncomment the **map_server** parameter section
- Enter the correct map file location:
  ```yaml
  map_server:
    yaml_filename: "/home/<your_username>/my_ws/src/slam/maps/nav_map.yaml"
  ```
- Save and close the file

2. **Launch NAV2 Stack**
```bash
source ~/my_ws/install/setup.bash
ros2 launch navigation nav2.launch.py
```

3. **Set Initial Pose**
- In RViz2, use the **2D Pose Estimate** tool to set the robot's initial position on the map
- This helps the robot localize itself within the pre-built map

4. **Navigate the Robot**
- Use the **2D Goal Pose** tool in RViz2 to set a goal position
- The robot will plan and execute an optimal path to the goal using the saved map

---

## Navigation Features

- **Autonomous Path Planning**: NAV2 stack computes optimal paths avoiding obstacles
- **Dynamic Obstacle Avoidance**: Real-time replanning when obstacles are detected
- **SLAM Integration**: Simultaneous localization and mapping capabilities
- **Localization**: AMCL-based localization for navigation with pre-built maps
- **Cost Maps**: Global and local costmap generation for safe navigation

## Credits & References

**Robot Model**: This project uses a URDF model adapted from the TortoiseBot example in the [OSRF ROS Book](https://github.com/osrf/rosbook/blob/master/code/tortoisebot/tortoisebot.urdf). The original model has been modified for ROS 2 Jazzy and Gazebo Harmonic integration with Nav2 stack capabilities.

**Original Authors**: Open Source Robotics Foundation (OSRF)

**Navigation Stack**: This project utilizes the ROS 2 Navigation (Nav2) stack for autonomous navigation and path planning. Nav2 is an open-source project maintained by the ROS 2 community. For more information, documentation, and tutorials, visit the [Nav2 Documentation](https://docs.nav2.org/)
