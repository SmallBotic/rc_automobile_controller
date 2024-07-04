# RC AUTOMOBILE CONTROLLER

This project is a simplified remote control for an RC Automobile base on the ESP32 microcontroller, MicroROS framework and the Arduino development environment.

This project is the control for the automobile in the [rc-automobile-ros](https://www.github.com/SmallBotic/rc-automobile-ros) project.

## Setup

For the hardware setup, read the [README.md](https://www.github.com/SmallBotic/rc-automobile-ros/#ros-automobile) file in the [rc-automobile-ros](https://www.github.com/SmallBotic/rc-automobile-ros) project.

***

For the software setup, follow the steps below:

1. Clone the repository:

    **NOTE:** _clone this repository in the `src` folder of your ROS workspace._

    ```bash
    git clone https://www.github.com/SmallBotic/rc-automobile-controller
    ```

2. Edit the [config.py](rc_automobile_controller/config.py) file to match your setup. This settings should be the same as the one in the [Config.hpp](https://www.github.com/SmallBotic/rc-automobile-ros/blob/main/include/Config.hpp) file in the [rc-automobile-ros](https://www.github.com/SmallBotic/rc-automobile-ros) project.

3. Build and source the workspace:

    ```bash
    colcon build
    source install/setup.zsh # or .bash if you are using bash
    ```

4. Run the project:

    ```bash
    ros2 run rc_automobile_controller rc_automobile_controller
    ```
