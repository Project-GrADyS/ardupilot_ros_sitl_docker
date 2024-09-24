# Ardupilot ROS-SITL Docker


## 📝 Overview
This guide provides instructions for setting up a complete ROS environment for interacting with Ardupilot using both MAVROS (MAVLink) and DDS (Data Distribution Service). The goal is to enable efficient communication and control of autonomous vehicles using ROS, leveraging the high-rate data transfer capabilities of DDS compared to MAVLink.

## 📋 Table of Contents
1. [Overview](#-overview)
2. [Technologies](#️-technologies)
3. [Repositories](#-repositories)
4. [Introduction](#-introduction)
5. [Installation](#-installation)
6. [Usage](#-usage)
   - [Running Docker Compose](#running-docker-compose)
   - [Simulation and Vehicle Connection Modes](#simulation-and-vehicle-connection-modes)
   - [Running Manually](#running-manually)
     - [ROS Humble](#ros-humble)
       - [Build the Image](#build-the-image)
       - [Running ROS](#running-ros)
       - [Experimental Mode](#experimental-mode)
     - [SITL and MAVProxy](#sitl-and-mavproxy)
       - [Build the Image](#build-the-image-1)
       - [Running SITL](#running-sitl)
7. [Configuration](#-configuration)
   - [Configuring ROS Deployment](#configuring-ros-deployment)
   - [Enabling Graphical Interface](#enabling-graphical-interface)



## 🛠️ Technologies
The setup is based on the following technologies:
- **Docker**: Used to containerize and run the Ardupilot and ROS environment.
- **ROS Humble**: For robotic system development.
- **MAVROS2**: ROS package to communicate with MAVLink-compatible autopilots.
- **DDS**: Data Distribution Service, providing faster communication than MAVLink.

## 📁 Repositories
The setup is based on the following repositories:

- SITL (Software in the Loop Simulation):
  - [ardupilot-sitl-docker](https://github.com/radarku/ardupilot-sitl-docker)
- ROS Humble (with MAVROS2):
  - [ardupilot_dev_docker](https://github.com/ArduPilot/ardupilot_dev_docker/tree/master)
- Ardupilot Copter-4.3:
  - [Ardupilot Copter-4.3](https://github.com/ArduPilot/ardupilot/tree/Copter-4.3)
- AP_DDS (DDS integration with Ardupilot):
  - [AP_DDS](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies)


## 💡 Introduction
### Setting Up ROS, MAVROS, and DDS Environment for Ardupilot
Configuring a functional ROS (Robot Operating System) environment to interact with Ardupilot can be quite challenging, especially when attempting to integrate MAVROS (MAVLink) and DDS (Data Distribution Service). Both of these systems serve critical roles in autonomous vehicle control, but they use different communication protocols, leading to complexities when switching or combining their use.

MAVROS relies on MAVLink, the well-established protocol for drone communication, widely adopted in the UAV industry. It is robust but operates at a slower rate of around 50 Hz. On the other hand, DDS, such as Eprosima XRCE-DDS, offers faster data transmission (above 200 Hz), which is particularly useful in scenarios that demand low-latency, high-frequency communication for autonomous vehicles. The challenge arises in configuring both MAVROS and DDS to work seamlessly within the same ROS environment, especially when transitioning between simulation modes and real-world vehicle deployment.

Migrating between these two environments—MAVROS for traditional control and DDS for high-speed communication—requires careful management of parameters and connection settings, particularly when switching from the slower, more stable MAVLink to the faster, more dynamic DDS protocol. Misconfigurations can lead to communication mismatches, making the setup fragile if not properly handled.

In this guide, we’ll walk through setting up this integrated environment, including Docker container configuration, simulator execution, and Ardupilot integration. We'll also address the nuances of deploying MAVROS and DDS for both simulated and real-world applications.

>DDS Middleware (Eprosima XRCE-DDS): For more efficient communication with lower latency, use XRCE-DDS. More information can be found [here](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies).

>MAVROS (MAVLink): For more traditional communication with Ardupilot via MAVLink, use MAVROS within ROS. More information can be found [here](https://github.com/mavlink/mavros)


[Learn more about the performance comparison between MAVLink and DDS here.](https://cdck-file-uploads-global.s3.dualstack.us-west-2.amazonaws.com/business7/uploads/ros/original/3X/f/1/f10479eaf0434928929bf0637f52a468102f6a51.pdf)

## 🚀 Installation
To get started with this environment, follow these steps:

1. Make sure [Docker](https://docs.docker.com/engine/install/) is installed on your machine.
2. Clone [this repository](https://github.com/Project-GrADyS/ardupilot_ros_sitl_docker)

## 🔧 Usage

### Running Docker Compose
To start the entire environment automatically, use Docker Compose in the **ARDUPILOT_ROS_SITL_DOCKER** folder by running the following command:

```sh
docker compose up
```

#### Simulation and Vehicle Connection Modes
You can configure whether to run the SITL (Software In The Loop) simulator or connect to a real vehicle via the serial bus by setting the SIM_MODE environment variable. Configure it as follows:
- To run the simulator, set `SIM_MODE=true`.
- To connect to a real vehicle, set `SIM_MODE=false`.

> *example*: `SIM_MODE=true docker compose up`

### Running Manually

If you prefer to execute each component manually, follow these steps:

#### ROS Humble
##### Build the Image:

To create the Docker image with ROS and MAVROS, use the provided Dockerfile:
```sh
docker run -it --rm ardupilot_ros
```

##### Running ROS:
To run the ROS Humble container with MAVROS, use the following command:

```sh
docker run -it --rm ardupilot_ros
```
##### Experimental Mode:
If you wish to run ROS in a custom Docker network and mount directories for development, use the following command. This will create a network called `ros_sitl` and map the source code to facilitate development.

```sh
docker run -it --rm --network ros_sitl --mount type=bind,source="$(pwd)"/PKG,target=/ros2_ws/src --mount type=bind,source="$(pwd)",target=/ros2_config_app,readonly ardupilot_ros

ros2 launch mavros apm.launch fcu_url:=tcp://sitl_1:5760

```
#### SITL and MAVProxy
##### Build the Image:
Build the Docker image for SITL:
```sh
docker build --tag ardupilot .
```
##### Running SITL:
To run the SITL simulator and enable MAVLink communication, execute the following command:

```sh
docker run -it --rm -p 5760:5760 ardupilot
```

To run SITL in a network with ROS:
```sh
docker run -it --rm --name sitl_1 --network ros_sitl -p 5760:5760 ardupilot
```

## ⚙️ Configuration

### Configuring ROS Deployment
In the **ardupilot_ros_docker/configs/config.ini** directory, you'll find a config.ini file. This configuration file is crucial for setting up the ROS deployment, where you can define several options related to the ROS execution, such as simulation modes, communication ports, and vehicle-specific parameters.

### Enabling Graphical Interface
If you need to visualize the simulator's graphical interface, ensure you run the following command before starting the Docker containers:

```sh
xhost +
```
This will allow Docker to access the host's graphical interface, which is essential for visualizing the simulation environment.