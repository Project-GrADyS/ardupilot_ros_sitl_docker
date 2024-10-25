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
        - [Mapping Serial Device in Docker Compose](#mapping-serial-device-in-docker-compose)
        - [Serial Device Permissions](#serial-device-permissions)
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

Configuring a functional ROS (Robot Operating System) environment to interact with ArduPilot presents a unique set of challenges, especially when integrating MAVROS (MAVLink) and DDS (Data Distribution Service). Both of these systems are essential for the effective control of autonomous vehicConfiguring a functional ROS (Robot Operating System) environment to interact with ArduPilot presents a unique set of challenges, especially when integrating MAVROS (MAVLink) and DDS (Data Distribution Service). Both of these systems are essential for the effective control of autonomous vehicles, but they operate on different communication protocols, which can complicate their combined use.

ArduPilot is a versatile autopilot system used across various types of vehicles, including drones, rovers, and boats. It provides a robust platform for developing autonomous applications. To maximize its capabilities, developers can leverage the extensive functionality of ROS alongside ArduPilot, allowing for more complex interactions and control strategies.

MAVROS utilizes MAVLink, a widely adopted communication protocol in the UAV industry. While MAVLink is reliable, it operates at a frequency of approximately 50 Hz, which may be insufficient for applications requiring rapid response times. In contrast, DDS, particularly with Eprosima XRCE-DDS, supports higher data transmission rates exceeding 200 Hz. This is particularly beneficial for applications demanding low-latency, high-frequency communication, such as real-time control in autonomous vehicles.

Integrating MAVROS and DDS within a single ROS environment necessitates meticulous management of configuration parameters and connection settings. Transitioning between MAVLink's more stable yet slower communication and DDS's dynamic capabilities can lead to communication mismatches if not handled properly. This fragility underscores the importance of a well-structured setup.

To streamline the development process, containerization offers a practical solution. By utilizing Docker containers, developers can create isolated environments tailored for both MAVROS and DDS, simplifying the deployment of scripts designed to control ArduPilot vehicles. This approach allows for easy version management, testing, and collaboration, making it easier to switch between different configurations and environments.

In this guide, we will detail the steps for setting up an integrated environment, including Docker container configuration, simulator execution, and ArduPilot integration. Additionally, we will explore the nuances of deploying MAVROS and DDS for both simulated and real-world applications, providing a comprehensive understanding of how to harness these technologies effectively.les, but they operate on different communication protocols, which can complicate their combined use.

ArduPilot is a versatile autopilot system used across various types of vehicles, including drones, rovers, and boats. It provides a robust platform for developing autonomous applications. To maximize its capabilities, developers can leverage the extensive functionality of ROS alongside ArduPilot, allowing for more complex interactions and control strategies.

MAVROS utilizes MAVLink, a widely adopted communication protocol in the UAV industry. While MAVLink is reliable, it operates at a frequency of approximately 50 Hz, which may be insufficient for applications requiring rapid response times. In contrast, DDS, particularly with Eprosima XRCE-DDS, supports higher data transmission rates exceeding 200 Hz. This is particularly beneficial for applications demanding low-latency, high-frequency communication, such as real-time control in autonomous vehicles.

Integrating MAVROS and DDS within a single ROS environment necessitates meticulous management of configuration parameters and connection settings. Transitioning between MAVLink's more stable yet slower communication and DDS's dynamic capabilities can lead to communication mismatches if not handled properly. This fragility underscores the importance of a well-structured setup.

To streamline the development process, containerization offers a practical solution. By utilizing Docker containers, developers can create isolated environments tailored for both MAVROS and DDS, simplifying the deployment of scripts designed to control ArduPilot vehicles. This approach allows for easy version management, testing, and collaboration, making it easier to switch between different configurations and environments.

In this guide, we will detail the steps for setting up an integrated environment, including Docker container configuration, simulator execution, and ArduPilot integration. Additionally, we will explore the nuances of deploying MAVROS and DDS for both simulated and real-world applications, providing a comprehensive understanding of how to harness these technologies effectively.

>DDS Middleware (Eprosima XRCE-DDS): For more efficient communication with lower latency, use XRCE-DDS. More information can be found [here](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies).

>MAVROS (MAVLink): For more traditional communication with Ardupilot via MAVLink, use MAVROS within ROS. More information can be found [here](https://github.com/mavlink/mavros)

[Learn more about the performance comparison between MAVLink and DDS here.](https://cdck-file-uploads-global.s3.dualstack.us-west-2.amazonaws.com/business7/uploads/ros/original/3X/f/1/f10479eaf0434928929bf0637f52a468102f6a51.pdf)


## Archtecture
![Archtecture Diagram](images/archtecture%20diagram.png)

The architecture for implementing this solution consists of two Docker containers: one running a SITL (Software In The Loop) image and the other hosting a ROS 2 image. This setup is designed to facilitate the development and testing of autonomous vehicle code in a controlled environment.

The SITL container simulates the flight dynamics of an ArduPilot-controlled vehicle, allowing developers to test their code without the need for physical hardware. Within this container, a standard network, referred to as ros_sitl_net, is established to facilitate communication between the SITL and the ROS 2 environment. This network configuration ensures that messages and data can be exchanged seamlessly during testing.

The ROS 2 container serves as the primary development environment, where developers can create and run their autonomous vehicle applications. This setup not only allows for interaction with the SITL simulation but also supports connections to physical interfaces, enabling developers to test their code with real-world hardware when necessary.

The entire process is orchestrated using Docker Compose, which simplifies the management of multi-container applications. With Docker Compose, developers can easily configure and launch both containers, ensuring they are connected and functioning correctly within the specified network. This streamlined workflow enhances the development process by enabling rapid iteration and testing, ultimately accelerating the deployment of autonomous vehicle solutions.

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

#### Mapping Serial Device in Docker Compose

When configuring serial devices for communication with the flight controller (FCU), it's important to map the correct serial device in your `docker-compose.yaml` file. For example:

```yaml
services:
  ros-control:
    devices:
      - /dev/ttyS0:/dev/ttyS0
```

In this case, `/dev/ttyS0` on the host is mapped to `/dev/ttyS0` inside the container. You should adjust this mapping according to the specific device you are using.

Additionally, don't forget to update the FCU_URL in the .env file of your Docker Compose setup to reflect the correct serial device. For example:

```env
FCU_URL=/dev/ttyS0:115200
```

This ensures that your ROS application communicates with the right serial port for the FCU.

#### Serial Device Permissions

When working with serial devices in Docker containers, it's crucial to ensure that the permissions are correctly set to avoid access issues.

##### Folder Structure

>**Warning:**  
Be cautious about the path you choose for storing data. It’s recommended **not to use `/home`** for data storage because user folders in `/home` tend to have restrictive permissions by default. This could lead to permission problems, creating unnecessary complexity. Instead, opt for directories outside `/home`, such as `/data`.

##### Permissions

To avoid permission issues, it's important to recursively set the correct ownership and permissions on the directories and files used in your Docker environment. Use the following commands to set the ownership and permissions:

```bash
sudo chown -R $USER:$USER /data/ardupilot_ros_sitl_docker
sudo chmod -R a=,a+rX,u+w,g+w /data/ardupilot_ros_sitl_docker
sudo chmod +x /data/ardupilot_ros_sitl_docker/ardupilot_ros_docker/configs/init.sh
```

### Running Manually

If you prefer to execute each component manually, follow these steps:

#### ROS Humble

##### Build the Image

To create the Docker image with ROS and MAVROS, use the provided Dockerfile:

```sh
docker run -it --rm ardupilot_ros
```

##### Running ROS

To run the ROS Humble container with MAVROS, use the following command:

```sh
docker run -it --rm ardupilot_ros
```

##### Experimental Mode

If you wish to run ROS in a custom Docker network and mount directories for development, use the following command. This will create a network called `ros_sitl` and map the source code to facilitate development.

```sh
docker run -it --rm --network ros_sitl --mount type=bind,source="$(pwd)"/PKG,target=/ros2_ws/src --mount type=bind,source="$(pwd)",target=/ros2_config_app,readonly ardupilot_ros

ros2 launch mavros apm.launch fcu_url:=tcp://sitl_1:5760

```

#### SITL and MAVProxy

##### Build the Image

Build the Docker image for SITL:

```sh
docker build --tag ardupilot .
```

##### Running SITL

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

In the root folder of your project, you will find the **.env** file. This configuration file is crucial for configuring your ROS deployment, where you can set various options related to running ROS, such as simulation modes, communication ports, and vehicle-specific parameters.

| **Option**            | **Definition**                                                    | **Usage**                                                                                        | **Example**                                     |
|---------------------- |------------------------------------------------------------------ |------------------------------------------------------------------------------------------------- |------------------------------------------------ |
| COMPILE_STATE         | determines whether there will be compilation                      | 0 -> OFF \| 1 -> ON                                                                              | COMPILE_STATE=0                                 |
| CREATE_PACKAGE_STATE  | determines whether packages will be created                       | 0 -> OFF \| 1 -> ON                                                                              | CREATE_PACKAGE_STATE=0                          |
| PACKAGES_TO_CREATE    | list of packages to be created                                    | "\<PKG1\> \<PKG2\> \<PKG3\>..."                                                                  | PACKAGES_TO_CREATE=drone_basics                 |
| PACKAGES_TO_BUILD     | list of packages to be compiled                                   | "\<PKG1\> \<PK2\> \<PKG3\>..."                                                                   | PACKAGES_TO_BUILD=drone_basics                  |
| NODES_FOR_RUN         | list of nodes to run. (requires the package the node belongs to)  | "\<PKG1\> \<NODE1\> \<PKG2\> \<NODE2\>..."                                                       | NODES_FOR_RUN="drone_basics random_fence_node"  |
| COM_ARCH              | defines the communication driver                                  | 1 -> Micro XRCE-DDS \| 2 -> MAVROS                                                               | COM_ARCH=2                                      |
| SIM_MODE              | determines whether execution is simulation or serial              | 0 -> OFF \| 1 -> ON                                                                              | SIM_MODE=1                                      |
| FCU_URL               | serial device (requires SIM_MODE disabled)                        | `/dev/<device>:<baudrate>`<br> or<br> `udp://<IP>:<PORT>@<PORT>`<br> or<br> `tcp://<IP>:<PORT>`  | FCU_URL=/dev/ttyS0:115200                       |
| TGT_SYSTEM            | Vehicle MAV_SYS_ID                                                | <int>                                                                                            | TGT_SYSTEM=23                                   |

### Enabling Graphical Interface

If you need to visualize the simulator's graphical interface, ensure you run the following command before starting the Docker containers:

```sh
xhost +
```

This will allow Docker to access the host's graphical interface, which is essential for visualizing the simulation environment.
