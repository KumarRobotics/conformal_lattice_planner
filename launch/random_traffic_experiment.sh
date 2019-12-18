#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT

method="ego_slc_lattice_planner"

episode_time=0.0
episode_timeout=0
max_episode_time=500.0

experiment_time=0.0
experiment_timeout=0
max_experiment_time=3600.0

while [ $experiment_timeout -le 0 ]; do
  # Start the carla server.
  echo "Start carla server."
  ./carla_server.sh
  sleep 10

  # Start the ros clients.
  echo "Start ROS nodes."
  roslaunch conformal_lattice_planner autonomous_driving.launch \
    random_traffic:=true \
    $method:=true \
    agents_lane_follower:=true \
    record_bags:=true&
  sleep 10

  # Keep track of the simulation time of this episode.
  while [ $episode_timeout -le 0 ]; do
    sleep 5
    current_time=$(rosservice call /carla/carla_simulator/simulation_time | tr '\n' ' ' | cut -d' ' -f4 | cut -d'"' -f2)
    if [ ${#current_time} -le 0 ]; then
      echo "ROS service /carla/carla_simulator/simulation_time unavailable"
      break;
    else
      episode_time=$current_time
    fi

    episode_timeout=$(echo "$episode_time>$max_episode_time" | bc -l)
    echo "episode time=$episode_time episode timeout=$episode_timeout"
    echo "experiment time=$experiment_time experiment timeout=$experiment_timeout"
  done

  # Update the total simulation time
  experiment_time=$(echo "$experiment_time+$episode_time" | bc -l)
  experiment_timeout=$(echo "$experiment_time>$max_experiment_time" | bc -l)
  episode_time=0.0
  episode_timeout=0
  echo "total time=$experiment_time experiment timeout=$experiment_timeout"

  # Kill all ROS nodes in case some nodes has not died yet.
  echo "Stop all ROS nodes"
  rosnode kill -a
  sleep 10

  # Kill the carla server.
  # FIXME: Need to restart and kill again.
  echo "Stop carla server"
  kill $(pidof CarlaUE4-Linux-Shipping)
  sleep 5
  ./carla_server.sh
  kill $(pidof CarlaUE4-Linux-Shipping)
  sleep 5

done
