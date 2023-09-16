# ros2-factory-sim
 An integrated simulation of a factory environment using ROS2, PlanSys2, Nav2, and Isaac Sim. Features three robots executing smart navigation and package handling plans.
 
This README.md provides step-by-step instructions for setting up and using the project. Follow the guidelines below to install dependencies, configure your environment, and avoid known bugs.

## Installation

Before proceeding, ensure you have [ROS Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/) and Isaac Sim installed on your system. Then, follow these steps to install the necessary dependencies:

1. Install required packages:

    ```bash
    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup
    sudo apt install ros-foxy-plansys2-*
    rosdep update --include-eol-distros
    ```

2. Navigate to your workspace directory:

    ```bash
    cd <workspace>
    ```

3. Install ROS dependencies:

    ```bash
    rosdep install -i --from-path src --rosdistro foxy -y
    ```

## Environment Setup

To ensure a smooth development experience, follow these steps each time you start a new shell session from within the workspace folder:

1. Clear the `LD_LIBRARY_PATH` variable:

    ```bash
    unset LD_LIBRARY_PATH
    ```

2. Set the `FASTRTPS_DEFAULT_PROFILES_FILE` variable:

    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
    ```

3. Source ROS Foxy:

    ```bash
    source /opt/ros/foxy/setup.bash
    ```

4. Source the local workspace setup:

    ```bash
    source install/local_setup.bash
    ```

You can also just add these commands to the bashrc file.

## Building

If you make any changes to the code, rebuild the project using the following command:

```bash
colcon build
```

## Known Bugs

colcon build --symlink-install
ros2 launch carter_navigation multiple_robot_carter_navigation_office.launch.py
ros2 launch plansys2_bt plansys2_bt_launch.py
ros2 run plansys2_terminal plansys2_terminal


### Rviz Crash

Do not enable the controller in RViz due to a segmentation fault issue. Enabling the controller will lead to a program crash. This issue is detailed in [this GitHub issue](https://github.com/ros2/rviz/issues/703). By default, the controller is disabled in RViz to prevent crashes.

Feel free to contribute and report any additional issues!
