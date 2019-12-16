#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT

simulation_time=0.0
episode_timeout=0

total_simulation_time=0.0
max_experiment_time=100.0
experiment_timeout=0

while [ ${experiment_timeout} -le 0 ]; do
  # Start the carla server.
  echo "Start carla server."
  ./carla_server.sh &
  sleep 30

  # Start the ros clients.
  echo "Start ROS nodes."
  roslaunch conformal_lattice_planner autonomous_driving.launch random_traffic:=true ego_slc_lattice_planner:=true agents_lane_follower:=true record_bags:=true&
  sleep 15

  # Keep track of the simulation time of this episode.
  while [ ${episode_timeout} -le 0 ]; do
    # If the nodes are no longer alive, stop the current episode.
    if [ -z `rosservice list | grep simulation_time` ]; then
      break
    fi
    simulation_time=`rosservice call /carla/carla_simulator/simulation_time | tr '\n' ' ' | cut -d' ' -f4 | cut -d'"' -f2`
    episode_timeout=`echo "$simulation_time>$max_experiment_time" | bc -l`
    echo "episode simulation time = ${simulation_time} ${episode_timeout}"
    sleep 5
  done

  # Update the total simulation time
  total_simulation_time=`python -c "print $total_simulation_time + $simulation_time"`
  experiment_timeout=`echo "$total_simulation_time>$max_experiment_time" | bc -l`
  echo "total simulation time = ${total_simulation_time} ${experiment_timeout}"

  # Kill the carla server.
  kill `pidof CarlaUE4-Linux-Shipping`
  sleep 5

done
