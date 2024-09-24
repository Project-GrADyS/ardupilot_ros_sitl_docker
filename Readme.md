# Table of Contents
1. [Setting Up ROS, MAVROS, and DDS Environment for Ardupilot](#setting-up-ros-mavros-and-dds-environment-for-ardupilot)
2. [Based on the Following Repositories](#based-on-the-following-repositories)
3. [Introduction](#introduction)
4. [Configuring ROS Deployment](#configuring-ros-deployment)
5. [Running Docker Compose](#running-docker-compose)
6. [Simulation and Vehicle Connection Modes](#simulation-and-vehicle-connection-modes)
7. [Enabling Graphical Interface](#enabling-graphical-interface)
8. [Running Manually](#running-manually)
   - [ROS Humble](#ros-humble)
     - [Build the Image](#build-the-image)
     - [Running the Image](#running-the-image)
     - [Experimental Mode](#experimental-mode)
   - [SITL and MAVProxy](#sitl-and-mavproxy)
     - [Build the Image](#build-the-image-1)
     - [Running SITL](#running-sitl)



# Setting Up ROS, MAVROS, and DDS Environment for Ardupilot
Configuring a fully functional ROS (Robot Operating System) environment to interact with Ardupilot via both MAVROS and DDS can be challenging due to the complexity of integrating these systems. ROS, the core software for controlling autonomous robots, must communicate efficiently with Ardupilot. This can be done either via the MAVLink protocol using MAVROS or through DDS (Data Distribution Service) middleware, such as Eprosima XRCE-DDS, which offers faster and more efficient communication.

The main difficulty lies in bridging these two worlds: MAVROS and DDS. While MAVROS uses MAVLink, the standard communication protocol for drones and aerial vehicles, XRCE-DDS provides faster communication, reaching rates of 200 Hz or more, significantly higher than MAVLink’s usual 50 Hz. Below is a step-by-step guide for setting up and running this environment, including Docker container setup, simulator execution, and Ardupilot integration.

>DDS Middleware (Eprosima XRCE-DDS): For more efficient communication with lower latency, use XRCE-DDS. More information can be found [here](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies).

>MAVROS (MAVLink): For more traditional communication with Ardupilot via MAVLink, use MAVROS within ROS. More information can be found [here](https://github.com/mavlink/mavros)


## Based on the Following Repositories:
- SITL (Software in the Loop Simulation):
  - [ardupilot-sitl-docker](https://github.com/radarku/ardupilot-sitl-docker)
- ROS Humble (with MAVROS2):
  - [ardupilot_dev_docker](https://github.com/ArduPilot/ardupilot_dev_docker/tree/master)
- Ardupilot Copter-4.3:
  - [Ardupilot Copter-4.3](https://github.com/ArduPilot/ardupilot/tree/Copter-4.3)
- AP_DDS (DDS integration with Ardupilot):
  - [AP_DDS](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies)

## Introduction

This setup aims to leverage DDS communication, allowing for faster data rates compared to MAVLink, especially in applications that require rapid responses in autonomous vehicles. XRCE-DDS can operate at rates above 200 Hz, whereas MAVLink is capped at around 50 Hz. Using ROS to communicate with Ardupilot, either through MAVROS or DDS, enables greater flexibility and performance when controlling drones and autonomous vehicles.

[Learn more about the performance comparison between MAVLink and DDS here.](https://cdck-file-uploads-global.s3.dualstack.us-west-2.amazonaws.com/business7/uploads/ros/original/3X/f/1/f10479eaf0434928929bf0637f52a468102f6a51.pdf)

## Configuring ROS Deployment
In the **ardupilot_ros_docker/configs/config.ini** directory, you'll find a config.ini file. This configuration file is crucial for setting up the ROS deployment, where you can define several options related to the ROS execution, such as simulation modes, communication ports, and vehicle-specific parameters.

## Running Docker Compose
To start the entire environment automatically, use Docker Compose by running the following command:

```sh
docker compose up
```

## Simulation and Vehicle Connection Modes
You can configure whether to run the SITL (Software In The Loop) simulator or connect to a real vehicle via the serial bus by setting the SIM_MODE environment variable. Configure it as follows:
- To run the simulator, set `SIM_MODE=true`.
- To connect to a real vehicle, set `SIM_MODE=false`.

> *example*: `SIM_MODE=true docker compose up`

## Enabling Graphical Interface
If you need to visualize the simulator's graphical interface, ensure you run the following command before starting the Docker containers:

```sh
xhost +
```
This will allow Docker to access the host's graphical interface, which is essential for visualizing the simulation environment.

## Running Manually
If you prefer to execute each component manually, follow these steps:

### ROS Humble
#### Build the Image:

To create the Docker image with ROS and MAVROS, use the provided Dockerfile:
```sh
docker run -it --rm ardupilot_ros
```

#### Running the Image:
To run the ROS Humble container with MAVROS, use the following command:

```sh
docker run -it --rm ardupilot_ros
```
#### Experimental Mode:
If you wish to run ROS in a custom Docker network and mount directories for development, use the following command. This will create a network called `ros_sitl` and map the source code to facilitate development.

```sh
docker run -it --rm --network ros_sitl --mount type=bind,source="$(pwd)"/PKG,target=/ros2_ws/src --mount type=bind,source="$(pwd)",target=/ros2_config_app,readonly ardupilot_ros

ros2 launch mavros apm.launch fcu_url:=tcp://sitl_1:5760

```
### SITL and MAVProxy
**Build the Image:**
Build the Docker image for SITL:
```sh
docker build --tag ardupilot .
```
#### Running SITL:
To run the SITL simulator and enable MAVLink communication, execute the following command:

```sh
docker run -it --rm -p 5760:5760 ardupilot
```

To run SITL in a network with ROS:
```sh
docker run -it --rm --name sitl_1 --network ros_sitl -p 5760:5760 ardupilot
```