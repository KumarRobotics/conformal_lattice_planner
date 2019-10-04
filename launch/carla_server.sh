#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT

CWD=`pwd`
cd ${Carla_DIST}

# Start Carla server without display
echo "Start CARLA server (-opengl -quality-level=low)."
DISPLAY=
./CarlaUE4.sh -opengl -quality-level=low &
SERVER_PID=$!

sleep 5s

# Change the the map
echo "Change map to Town04."
cd PythonAPI/util
python config.py --map Town04

sleep 3s

# Disable rendering
echo "Disable rendering of the CARLA server (GPU related data won't work)."
python config.py --no-rendering

sleep 3s

# Set the simulation step
echo "Simulation step is set to 0.05s (20Hz)."
python config.py --delta-seconds 0.05

# Do not exit until
echo "CARLA server initialization finishes."
echo "Ctrl+c to terminate the server."
wait ${SERVER_PID}

cd ${CWD}
