# based on Repositories: 
- SITL
    - https://github.com/radarku/ardupilot-sitl-docker
- ROS HUMBLE (with MAVROS2):
    - https://github.com/ArduPilot/ardupilot_dev_docker/tree/master
- Ardupilot Copter-4.3: 
    - https://github.com/ArduPilot/ardupilot/tree/Copter-4.3
- AP_DDS:
    - https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies

# Introduction
- rate limit XRCE DDS(200Hz+) is faster than MAVLink(50Hz)
- https://cdck-file-uploads-global.s3.dualstack.us-west-2.amazonaws.com/business7/uploads/ros/original/3X/f/1/f10479eaf0434928929bf0637f52a468102f6a51.pdf

# RUNNING COMPOSE
```sh
docker compose up
```

# RUNING MANUALLY:
## ROS HUMBLE
### BUILD IMAGE:
```sh
docker image build -t ardupilot_ros -f Dockerfile_dev-ros .
```

### RUN:
```sh
docker run -it --rm ardupilot_ros
```

#### EXPERIMENTAL:
```sh
docker run -it --rm --network ros_sitl --mount type=bind,source="$(pwd)"/PKG,target=/ros2_ws/src --mount type=bind,source="$(pwd)",target=/ros2_config_app,readonly ardupilot_ros

172.28.0.2:5760

ros2 launch mavros apm.launch fcu_url:=tcp://sitl_1:5760
```

## SITL AND MAVPROXY INSTANCE
### BUILD IMAGE:

```sh
docker build --tag ardupilot .
```
### RUN SITL:
```sh
docker run -it --rm -p 5760:5760 ardupilot
docker run -it --rm --name sitl_1 --network ros_sitl -p 5760:5760 ardupilot
```
### RUN MAVPROXY:
```sh
docker run -it --rm --name MAVProxy --network ros_sitl --entrypoint python3 ardupilot /home/dockeruser/.local/lib/python3.10/site-packages/MAVProxy/mavproxy.py --master=tcp:sitl_1:5760

```

# COMMUNICATION WITH AUTOPILOT POSSIBILITIES:
- DDS MIDDLEWARE (Eprosima XRCE-DDS) https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies
- MAVROS(MAVLINK)