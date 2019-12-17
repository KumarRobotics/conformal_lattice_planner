#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT

simulation_time=0.0
episode_timeout=0

total_simulation_time=0.0
max_experiment_time=3600.0
experiment_timeout=0

while [ ${experiment_timeout} -le 0 ]; do
  # Start the carla server.
  echo "Start carla server."
  ./carla_server.sh &
  carla_server_pid=$!
  sleep 30

  # Start the ros clients.
  echo "Start ROS nodes."
  roslaunch conformal_lattice_planner autonomous_driving.launch random_traffic:=true ego_slc_lattice_planner:=true agents_lane_follower:=true record_bags:=true&
  sleep 15

  # Keep track of the simulation time of this episode.
  while [ ${episode_timeout} -le 0 ]; do
    # If the nodes are no longer alive, stop the current episode.
    res=`rosnode ping -c 1 /carla/carla_simulator | grep "cannot ping" `
    if [ ${#res} -gt 0 ]; then
      echo "Simulator node died."
      break
    fi

    res=`rosnode ping -c 1 /carla/agents_lane_following_planner | grep "cannot ping"`
    if [ ${#res} -gt 0 ]; then
      echo "Agents planner node died."
      break
    fi

    res=`rosnode ping -c 1 /carla/ego_slc_lattice_planner | grep "cannot ping"`
    if [ ${#res} -gt 0 ]; then
      echo "Ego planner node died."
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

  echo "Stop all ROS nodes"
  rosnode kill -a
  sleep 10

  # Kill the carla server.
  echo "Stop carla server"
  kill $carla_server_pid
  sleep 10

done
