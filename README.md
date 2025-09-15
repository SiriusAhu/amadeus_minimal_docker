# Amadeus - Minimal ROS 2 Motion Control Container

<div align="center">

[![License: MIT](https://img.shields.io/github/license/SiriusAhu/amadeus_minimal_docker)](https://opensource.org/licenses/MIT)
![Amadeus](https://img.shields.io/badge/Team-Amadeus-green)

</div>

This repository provides a minimal ROS 2 container for the **Amadeus** smart car project at the Innovation Factory, XJTLU Taicang.

The goal of this project is to streamline the official motor control source code from [Hiwonder](https://www.hiwonder.com/) into a portable and standalone Docker image that contains only the core motion control functionalities.

Through a long but fruitful debugging process, we have resolved a wide range of issues, spanning from software dependencies and ROS communication to low-level hardware access permissions.

---

## Hardware Configuration

This project is built upon the **Raspberry Pi 5** and a smart car kit from **Hiwonder**. For more details, please refer to the links below:

- **Host Computer**: [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- **Motor Controller (Expansion Board)**: [Raspberry Pi 5 Expansion Board, Servo Motor Driver, ROS Robot Manipulator Arm Smart Car Controller](https://www.hiwonder.com/products/expansion-board-for-raspberry-pi-5?variant=40939498700887)
- **Motors**: [High Quality TT Motor with Plastic Shaft 3V-6V 150RPM Ratio 1:42 for Smart Car Robot](https://www.hiwonder.com/products/high-quality-tt-motor?variant=40452469751895&_pos=12&_sid=3d8a5578c&_ss=r)

---

## Quick Start

This project encapsulates all complex configurations and dependencies within Docker. You only need to have Docker installed on your host machine (e.g., Raspberry Pi).

### 1. Environment Setup

- **Clone the Repository**:
  ```bash
  git clone https://github.com/SiriusAhu/amadeus_minimal_docker

  cd amadeus_minimal_docker
  ```
- **Grant Execute Permissions to All Scripts**:
  ```bash
  chmod +x *.sh
  ```
- **Configure Hardware Device (Crucial!)**:
  Before launching, you must identify the serial device your robot's controller is connected to.
  1. In the host terminal, run `dmesg -w` and observe the newly appeared device name (e.g., `/dev/ttyAMA0` or `/dev/ttyACM0`).
  3. Open the `start.sh` file and modify the `HOST_DEVICE` variable to match your device name.

### 2. Launch and Test

A set of convenient management scripts is provided to simplify all operations.

#### Start Core Services
In one terminal, run `start.sh`. This script will automatically perform all the following tasks:
1.  Build the latest Docker image (`amadeus:minimal`) using the `Dockerfile`.
2.  Start the container in **privileged mode** (`--privileged`) to resolve hardware access permission issues (**especially important**).
3.  Correctly map your physical serial device to `/dev/rrc` inside the container.
4.  Launch the ROS 2 motion control nodes within the container.

```bash
# Run in Terminal 1
./start.sh
```
You will see all ROS nodes start successfully and wait for commands.

#### Test Hardware Connection (Buzzer)
Open a **second terminal** and run `test_buzzer.sh`. This script bypasses ROS to directly invoke a low-level SDK command, making the hardware buzzer beep. It's the best way to verify that the physical connection is solid.
```bash
# Run in Terminal 2
./test_buzzer.sh
```
If you hear a beep, the hardware connection is perfect!

#### Test Motor Movement
In the **second terminal**, run `test_motor.sh`. This script also directly calls the SDK to make the motors run for one second and then stop.
```bash
# Run in Terminal 2
./test_motor.sh
```
If the wheels turn, your Amadeus is ready for commands!

### 3. Other Operations

#### Enter the Container
If you need to enter the container for debugging (e.g., using `ros2 topic list`), you can run `enter.sh` in the second terminal.
```bash
# Run in Terminal 2
./enter.sh
```

#### Stop the Container
When you are finished with your tests, you can press `Ctrl+C` in the first terminal to stop the container. If the container is running in the background, you can use `stop.sh` to stop it.
```bash
# Run in any terminal
./stop.sh
```

#### Data Persistence (Disabled by Default)

By default, the `start.sh` script automatically removes the container upon stopping (due to the `--rm` flag). This means any changes you make inside the container (e.g., installing new tools with `apt install`) will be lost. This is useful for maintaining a clean testing environment.

If you are developing and wish to save your changes, you can enable persistence mode:

1.  Open the `start.sh` file.
2.  Find the `ENABLE_PERSISTENCE` variable at the top of the script.
3.  Change its value from `"false"` to `"true"`.

When enabled, the script will automatically:
-   Create a Docker volume named `amadeus_ws_data` to save your workspace build artifacts and logs.
-   **Not** remove the container when it stops.

The next time you run `./start.sh`, Docker might inform you that a container named `amadeus_control` already exists. You can manually remove it first with `./stop.sh` and `docker rm amadeus_control` before starting again.

---

## Acknowledgments

The core low-level control code responsible for direct hardware communication with the Hiwonder board (located in the `src/controller`, `src/ros_robot_controller`, and `src/ros_robot_controller_msgs` directories) is derived from the official source code provided by Hiwonder.

The main contribution of this repository is to significantly streamline, refactor, debug, and Dockerize this source code to create a stable, portable, and minimal motion control environment.

We express our sincere gratitude to Hiwonder.

---

## License

This project is licensed under the [MIT License](LICENSE).