# Amadeus ROS 2 最小化运动控制容器

<!-- 居中 -->
<div align="center">

![License: MIT](https://img.shields.io/github/license/SiriusAhu/amadeus_minimal_docker)
![Amadeus](https://img.shields.io/badge/Team-Amadeus-green)

</div>

这是用于 XJTLU Taicang, Innovation Factory **Amadeus** 智能车项目的 ROS 2 最小化容器仓库。

这个项目旨在将来源于[幻尔（HiWonder）](https://www.hiwonder.com/)的电机控制源码精简为一个只包含核心运动控制功能的、可移植的、独立的 Docker 镜像。

我们通过一个漫长但成果丰硕的调试过程，最终解决了从软件依赖、ROS通信到硬件底层访问权限的所有问题。

---

## 硬件配置

我们使用了**树莓派5**和**幻尔（HiWonder）**的智能车套件来构建这个项目。详细信息请参考：

- **主机**: [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- **电机主控板（扩展板）**: [Raspberry Pi 5 Expansion Board, Servo Motor Driver, ROS Robot Manipulator Arm Smart Car Controller](https://www.hiwonder.com/products/expansion-board-for-raspberry-pi-5?variant=40939498700887)
- **电机**: [High Quality TT Motor with Plastic Shaft 3V-6V 150RPM Ratio 1:42 for Smart Car Robot](https://www.hiwonder.com/products/high-quality-tt-motor?variant=40452469751895&_pos=12&_sid=3d8a5578c&_ss=r)

---

## 快速开始

本项目已经将所有复杂的配置和依赖项打包进 Docker。你只需要确保你的主机（例如树莓派）已经安装了 Docker。

### 1. 环境准备

- **克隆仓库**:
  ```bash
  git clone https://github.com/SiriusAhu/amadeus_minimal_docker

  cd amadeus_minimal_docker
  ```
- **赋予所有脚本执行权限**:
  ```bash
  chmod +x *.sh
  ```
- **配置硬件设备 (重要!)**:
  在启动前，你需要确认你的机器人主控板连接到主机的哪个串口设备。
  1. 在主机终端运行 `dmesg -w` 并观察新出现的设备名（例如 `/dev/ttyAMA0` 或 `/dev/ttyACM0`）。
  3. 打开 `start.sh` 文件，修改 `HOST_DEVICE` 变量为你的设备名。

### 2. 启动与测试

我们提供了一套便捷的管理脚本来简化所有操作。

#### 启动核心服务
在一个终端中，运行 `start.sh`。这个脚本会自动完成以下所有工作：
1.  使用 `Dockerfile` 构建最新的 Docker 镜像 (`amadeus:minimal`)。
2.  以**特权模式** (`--privileged`) 启动容器，解决硬件访问权限问题。（**尤为重要**）
3.  将你的物理串口设备正确映射到容器内部的 `/dev/rrc`。
4.  启动容器内的 ROS 2 运动控制节点。

```bash
# 在终端 1 中运行
./start.sh
```
你会看到所有 ROS 节点成功启动并等待指令。

#### 测试硬件连接 (蜂鸣器)
打开**第二个终端**，运行 `test_buzzer.sh`。这个脚本会绕过 ROS，直接调用底层 SDK 命令硬件蜂鸣器响起。这是验证物理连接是否通畅的最佳方式。
```bash
# 在终端 2 中运行
./test_buzzer.sh
```
如果听到蜂鸣声，说明硬件连接完美！

#### 测试电机运动
在**第二个终端**，运行 `test_motor.sh`。这个脚本同样直接调用底层 SDK 命令电机转动1秒后停止。
```bash
# 在终端 2 中运行
./test_motor.sh
```
如果轮子转动，说明你的 Amadeus 已经准备好接收指令了！

### 3. 其他操作

#### 进入容器内部
如果你需要进入容器内部进行调试（例如使用 `ros2 topic list` 等命令），可以在第二个终端运行 `enter.sh`。
```bash
# 在终端 2 中运行
./enter.sh
```

#### 停止容器
当你完成测试后，可以在第一个终端按 `Ctrl+C` 来停止容器。如果容器在后台运行，你可以使用 `stop.sh` 来停止它。
```bash
# 在任意终端运行
./stop.sh
```

#### 数据持久化 (默认关闭)

默认情况下，`start.sh` 脚本会在容器停止时自动删除容器 (`--rm` 参数)，这意味着你在容器内做的任何修改（例如 `apt install` 安装新工具）都会丢失。这对于保持测试环境的纯净很有用。

如果你正在进行开发，希望保存你在容器内做的修改，可以启用持久化模式：

1.  打开 `start.sh` 文件。
2.  找到脚本顶部的 `ENABLE_PERSISTENCE` 变量。
3.  将它的值从 `"false"` 修改为 `"true"`。

启用后，脚本会自动：
-   创建一个名为 `amadeus_ws_data` 的 Docker 数据卷（volume），用于保存你的工作区编译产物和日志。
-   在容器停止后，**不会**自动删除它。

下次你再次运行 `./start.sh` 时，如果名为 `amadeus_control` 的旧容器还存在，Docker 会提示你。你可以先用 `./stop.sh` 并加上 `rm` 命令来手动删除它：`docker rm amadeus_control`，然后再启动。

---

## 致谢 (Acknowledgments)

本项目中与幻尔（HiWonder）硬件直接通信的核心底层控制代码（位于 `src/controller`, `src/ros_robot_controller`, 和 `src/ros_robot_controller_msgs` 目录中）来源于幻尔官方提供的源码。

本仓库主要工作是在此基础上进行了大量的精简、重构、调试和 Docker 化封装，以创建一个稳定、可移植的最小化运动控制环境。

我们对幻尔（HiWonder）表示诚挚的感谢。

---

## 许可证

本项目采用 [MIT License](LICENSE) 授权。