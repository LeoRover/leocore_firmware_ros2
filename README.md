# leocore_firmware_ros2

The firmware for the [LeoCore] controller running inside Leo Rover. 

The main functionalities include:
- velocity commands for the robot,
- velocity and PWM commands for individual wheels,
- battery voltage feedback,
- wheel states (position, velocity, torque, PWM duty) feedback,
- odometry feedback (calculated from wheel encoders),
- feedback from the IMU sensor.

The project is written for [PlatformIO] and uses the [STM32Cube] embedded software libraries. The low-level code is generated by the [STM32CubeMX] tool.

The firmware also uses [rcl+rclc] as a client library with [Micro XRCE-DDS] as a middleware to expose its functionalities on ROS topics, services and parameters. For the documentation of the ROS API, visit [leo_fw] on the ROS wiki.

## Prerequisites
To build the project, all you'll need is [Visual Studio Code] with the [PlatformIO IDE] extension.

To change the [STM32CubeMX] configuration, you will also need the [stm32pio] tool in order to properly regenerate the code.

## Building
Open the project in [PlatformIO IDE], then run the `PlatformIO: Build` task.

## Flashing
### Using ST-Link programmer
Connect the ST-Link to the pins on the [LeoCore] debug pin header, then run the `PlatformIO: Upload` task.

You can also run the GDB debugger by running the `PIO Debug` launch configuration (or just clicking `F5`).

### Using RPi on Leo Rover
Upload the `.pio/build/leocore/firmware.bin` to Leo Rover, then, on the robot, run:
```
ros2 run leo_fw flash firmware.bin
```

## Connecting
To expose the Micro-ROS node to the ROS2 network, you need to run the [Micro-ROS Agent] on RPi. Build the package using [colcon] and then run:
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/serial0 -b 460800
```

## Modifying STM32CubeMX configuration\
Open the `leocore.ioc` project in [STM32CubeMX], make the changes and save the project.

**Don't** click `GENERATE CODE`. Instead, type:
```
stm32pio generate
```
The current configuration assumes that [STM32CubeMX] is installed under `/opt/STM32CubeMX`. You can change the path in the `stm32pio.ini` file.

[LeoCore]: https://www.leorover.tech/documentation/leo-core
[leo_fw]: http://wiki.ros.org/leo_fw
[stm32pio]: https://github.com/ussserrr/stm32pio
[PlatformIO IDE]: https://platformio.org/platformio-ide
[PlatformIO]: https://docs.platformio.org/en/latest/what-is-platformio.html
[STM32Cube]: https://www.st.com/en/ecosystems/stm32cube.html
[STM32CubeMX]: https://www.st.com/en/development-tools/stm32cubemx.html
[Visual Studio Code]: https://code.visualstudio.com

[rcl+rclc]: https://github.com/ros2/rclc
[Micro XRCE-DDS]: https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/
[Micro-ROS Agent]: https://github.com/micro-ROS/micro-ROS-Agent
[colcon]: https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html