FROM ubuntu:22.04

ARG VEHICLE_TAG=Copter-4.3

# install git 
RUN apt-get update && apt-get install -y git; git config --global url."https://github.com/".insteadOf git://github.com/

# Now grab ArduPilot from GitHub
RUN git clone https://github.com/ArduPilot/ardupilot.git ardupilot
WORKDIR ardupilot

# Checkout the latest Copter...
# RUN git checkout ${VEHICLE_TAG}

# Now start build instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Trick to get apt-get to not prompt for timezone in tzdata
ENV DEBIAN_FRONTEND=noninteractive

# Need sudo and lsb-release for the installation prerequisites
RUN apt-get install -y sudo lsb-release tzdata

RUN useradd -m dockeruser && echo "dockeruser:dockeruser" | chpasswd && adduser dockeruser sudo

RUN echo 'dockeruser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV SKIP_AP_GIT_CHECK=1
RUN su -c "Tools/environment_install/install-prereqs-ubuntu.sh -y" dockeruser

RUN export PATH=$PATH:$HOME/ardupilot/Tools/autotest
RUN export PATH=/usr/lib/ccache:$PATH

RUN python -m pip install empy
RUN python -m pip install pexpect
RUN pip install future
RUN pip install MAVProxy
RUN python -m pip install pymavlink
RUN pip3 install empy==3.3.4 --user


# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter
RUN ./waf rover 
RUN ./waf plane
RUN ./waf sub

RUN export PATH=$PATH:/ardupilot/Tools/autotest

# TCP 5760 is what the sim exposes by default
EXPOSE 5760/tcp

# Variables for simulator
ENV INSTANCE 0
ENV LAT 42.3898
ENV LON -71.1476
ENV ALT 14
ENV DIR 270
ENV MODEL +
ENV SPEEDUP 1
ENV VEHICLE ArduCopter

# Finally the command
ENTRYPOINT /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --speedup ${SPEEDUP}
