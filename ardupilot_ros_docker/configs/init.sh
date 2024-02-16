#!/bin/bash

# echo colors
RED="\e[0;31m"
GREEN_BOLD="\e[1;32m"
BLUE_BOLD="\e[1;34m"
NC="\e[0;0m"

# Arquivo de configuração
CONFIG_FILE="/ros2_config_app/config.ini"

# Função para ler valores do arquivo INI
read_ini() {
    local section=$1
    local key=$2
    local value=$(awk -F '=' '/\['$section'\]/{f=1} f==1&&$1~/'$key'/{print $2;exit}' $CONFIG_FILE)
    echo "$value"
}

# Lendo e usando os valores do arquivo INI
PACKAGES_TO_CREATE=$(read_ini "Configs" "PACKAGES_TO_CREATE")
PACKAGES_TO_BUILD=$(read_ini "Configs" "PACKAGES_TO_BUILD")
NODES=$(read_ini "Configs" "NODES_FOR_RUN")
COMPILE_STATE=$(read_ini "Configs" "COMPILE_STATE")
CREATE_PACKAGE_STATE=$(read_ini "Configs" "CREATE_PACKAGE_STATE")

source /opt/ros/humble/setup.bash
mkdir -p /ros2_ws/src
cd /ros2_ws/src

#no lugar do git clone, devera conter um laco "for" de nome de importacoes dos pacotes
#git clone https://github.com/ros/ros_tutorials.git -b humble
#git clone https://github.com/ros-geographic-info/geographic_info.git
if [ $CREATE_PACKAGE_STATE -eq 1 ]; then
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

# aqui devera ter um laco "for" com o nome dos pacotes a serem compilados
if [ $COMPILE_STATE -eq 1 ]; then
    echo -e "${RED} \
    ********************************** \
    BUILDING PACKAGES \
    **********************************"
    rosdep update && rosdep install -i --from-path src --rosdistro humble -y
    for package_name in $PACKAGES_TO_BUILD; do
        echo -e "${RED}BUILDING ${GREEN_BOLD}$package_name"
        colcon build --packages-up-to $package_name --symlink-install
    done
fi
# --packages-up-to builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
# --symlink-install saves you from having to rebuild every time you tweak python scripts

source /opt/ros/humble/setup.bash

#aqui devera rodar o mavros(unstable)
echo -e "${RED}RUNNING ${GREEN_BOLD}MAVROS"
ros2 launch mavros apm.launch fcu_url:=tcp://sitl_1:5760

#aqui devera ter um laco "for" que ativara os conjuntos de nodes que irao rodar em ordem
#for node in $NODES; do
#    ros2 run $node
#done