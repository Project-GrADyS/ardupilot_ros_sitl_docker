#!/bin/bash

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
PACKAGES=$(read_ini "Configs" "PACKAGES")
NODES=$(read_ini "Configs" "NODES_FOR_RUN")
COMPILE_STATE=$(read_ini "Configs" "COMPILE_STATE")

source /opt/ros/humble/setup.bash
cd /ros2_ws/src

#no lugar do git clone, devera conter um laco "for" de nome de importacoes dos pacotes
#git clone https://github.com/ros/ros_tutorials.git -b humble
#git clone https://github.com/ros-geographic-info/geographic_info.git
cd ..
#rosdep update && rosdep install -i --from-path src --rosdistro humble -y #deve ser colocado 
#mavros devera ser empacotado/compilado no dockerfile

# aqui devera ter um laco "for" com o nome dos pacotes a serem compilados
if [ $COMPILE_STATE -eq 1 ]; then
    for package_name in $PACKAGES; do
        colcon build --packages-up-to $package_name --symlink-install
    done
fi
# --packages-up-to builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
# --symlink-install saves you from having to rebuild every time you tweak python scripts

#aqui devera rodar o mavros(unstable)


#aqui devera ter um laco "for" que ativara os conjuntos de nodes que irao rodar em ordem
#for node in $NODES; do
#    ros2 run $node
#done

#/bin/bash