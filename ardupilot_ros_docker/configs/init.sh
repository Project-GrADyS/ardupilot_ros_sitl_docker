#!/bin/bash

source constants.sh

# Função para ler valores do arquivo INI
read_ini() {
    local section=$1
    local key=$2
    local value=$(awk -F '=' '/\['$section'\]/{f=1} f==1&&$1~/'$key'/{print $2;exit}' "$CONFIG_FILE")
    echo "$value"
}

# Função para criar vetor de subvetores
create_subvectors() {
    local nodes="$1" # Recebe a string de entrada como argumento
    echo "$nodes" | awk '{ for (i=1; i<=NF; i+=2) printf "\"%s %s\" ", $i, $(i+1) }'
}

# Lendo e usando os valores do arquivo INI
PACKAGES_TO_CREATE=$(read_ini "Configs" "PACKAGES_TO_CREATE")
PACKAGES_TO_BUILD=$(read_ini "Configs" "PACKAGES_TO_BUILD")
NODES_FOR_RUN=$(read_ini "Configs" "NODES_FOR_RUN")
COMPILE_STATE=$(read_ini "Configs" "COMPILE_STATE")
CREATE_PACKAGE_STATE=$(read_ini "Configs" "CREATE_PACKAGE_STATE")
COM_ARCH=$(read_ini "Configs" "COM_ARCH")
FCU_URL=$(read_ini "Configs" "FCU_URL")

nodes_run_array=$(create_subvectors "$NODES_FOR_RUN")
eval "nodes_run_array=($nodes_run_array)"

source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p /ros2_ws/src
cd /ros2_ws/src

#no lugar do git clone, devera conter um laco "for" de nome de importacoes dos pacotes
#git clone https://github.com/ros/ros_tutorials.git -b humble
#git clone https://github.com/ros-geographic-info/geographic_info.git
if [ "$CREATE_PACKAGE_STATE" -eq 1 ]; then
    echo -e "${RED} \
    ********************************** \
    CREATING PACKAGES \
    **********************************"
    for package_name in $PACKAGES_TO_CREATE; do
        echo -e "${RED}CREATING ${GREEN_BOLD}$package_name ${NC}in ${BLUE_BOLD}$(pwd) ${NC}folder"
        ros2 pkg create --build-type ament_python --license Apache-2.0 $package_name
    done
fi
cd ..

function build_package() {
    echo -e "${RED}BUILDING ${GREEN_BOLD}$1"
    colcon build --packages-up-to $1 --cmake-clean-cache
    source install/local_setup.bash
}

# aqui devera ter um laco "for" com o nome dos pacotes a serem compilados
if [ "$COMPILE_STATE" -eq 1 ]; then
    echo -e "${RED} \
    ********************************** \
    BUILDING PACKAGES \
    **********************************"
    rosdep update && rosdep install -i --from-paths src --ignore-src --rosdistro humble -y
    for package_name in $PACKAGES_TO_BUILD; do
        build_package $package_name
    done
fi
# --packages-up-to builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
# --symlink-install saves you from having to rebuild every time you tweak python scripts

source /opt/ros/$ROS_DISTRO/setup.bash

if [ "$COM_ARCH" -eq 1 ]; then
    echo -e "${GREEN_BOLD}INITIALIZING-MODE:\
    ${RED}EProsima Micro XRCE DDS Client"
    if ! [ -d "./src/micro_ros_setup" ]; then

        git clone -b "$ROS_DISTRO" \
            https://github.com/micro-ROS/micro_ros_setup.git \
            src/micro_ros_setup

        apt update && rosdep update
        rosdep install --from-paths src --ignore-src -y
        build_package micro_ros_setup
        source install/local_setup.bash

        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh
    fi

    source /ros2_ws/install/local_setup.bash
    cd /
    #in development
    if [ ! -d "ardupilot" ]; then
        git clone --depth=1 https://github.com/ArduPilot/ardupilot.git
        cd /ardupilot || exit
        git sparse-checkout set --no-cone libraries/AP_DDS/
        cd /ardupilot/libraries/AP_DDS/ || exit
    fi
    #export PATH=$PATH:/dds-gen/scripts
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 144540 --verbose 6

fi
#https://discuss.ardupilot.org/t/testing-ppp-and-dds-on-a-desktop-or-laptop/120187
#aqui devera rodar o mavros(unstable)
if [ "$COM_ARCH" -eq 2 ]; then
    echo -e "${GREEN_BOLD}INITIALIZING MODE:\
    ${RED}MAVROS"
    ros2 launch mavros apm.launch fcu_url:="$FCU_URL" tgt_system:=1 &
fi
#fcu_url:=udp://sitl_1:14551@14551 tgt_system:=1
#fcu_url:=tcp://sitl_1:5760 tgt_system:=1

# Esperando um pouco para garantir que o MAVROS seja iniciado completamente antes de iniciar os nós ROS 2
sleep 8

#aqui devera ter um laco "for" que ativara os conjuntos de nodes que irao rodar em ordem
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
for package_node in "${nodes_run_array[@]}"; do
    echo -e "${RED}RUNNING ${GREEN_BOLD}$package_node"
    ros2 run $package_node
done

# Esperando todos os nós serem executados antes de terminar o script
wait
