#!/bin/bash

# echo colors
RED="\e[0;31m"
GREEN_BOLD="\e[1;32m"
BLUE_BOLD="\e[1;34m"
NC="\e[0;0m"


if [ ${COMMUNICATION_ARCH} -eq 1 ]; then
    echo -e "${GREEN_BOLD}INITIALIZING-MODE: \
    ${RED}EProsima Micro XRCE DDS Client"

    /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --enable-dds --speedup ${SPEEDUP}
fi

if [ ${COMMUNICATION_ARCH} -eq 2 ]; then
    echo -e "${GREEN_BOLD}INITIALIZING MODE: \
    ${RED}MAVROS"

    /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --speedup ${SPEEDUP}
fi