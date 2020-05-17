# Software Framework

The following figure presents an overview of the node structure of this repository.

![node structure](scripts/software_framework.png)

## Simulator Node

Traffic simulation nodes are the main interface between ROS and Carla server, responsible for configuring the Carla server, spawning and deleting actors, retrieving actor information, etc. Three different types of traffic simulations are implemented (see `src/node/simulator`):

* **No Traffic**: only the ego vehicle is simulated with no agent vehicles.
* **Fixed Scenario**: agent vehicles can be preset around the ego, which aims at producing reproducable results from motion planning algorithms.
* **Random Traffic**: A fixed number of agent vehicles are maintained around the ego vehicle. Based on the traffic state, new agent vehicles may be spawned or existing agent vehicles may be removed from the simulation.

## Agent Vehicle Planning Node

There is currently only one motion planning algorithm implemented for agent vehicles, which control all agent vehicles to follow their lanes. The speed of the vehicles are modulated through the intelligent driver model. See `src/node/planner/agents_lane_following_node.h` for more details.

## Ego Vehicle Planning Node

Several lattice motion planning algorithms are implemented for the ego vehicle.

* Lane following: control the ego vehicle to follow the current lane.
* IDM lattice planning: see [Ke RAL 2020] for more details.
* Single-lane-change lattice planning: this is mostly a variation of IDM lattice planning. Over the spatial planning horizon, only a single lane change is allowed for the ego vehicle.
* Spatiotemporal lattice planning: a primitive implementation of the work from [[McNaughton ICRA 2011]](https://ieeexplore.ieee.org/abstract/document/5980223?casa_token=-Y27ZPo4PIUAAAAA:UQVVS0j-gVQgGdA1QU-On6icfexBZOGnOjzXSo6IJnxMp0bg7rdTXmCkPYU-C6-Y3riD9_mZ) for more details.

# Usage

All launch files can be found in the `launch` directory. To launch the simulation, start with,
```
./carla_server.sh
```
This script starts and configs the Carla simulator server. Note only the map Town04 is supported, otherwise the user has to re-define the route to be followed by the vehicle. For now, we hard-coded the road sequence to be followed by the vehicles in Town04. See `src/router/loop_router` for more details.

Use different launch files to start different types of simulation nodes and planning nodes for agent and ego vehicles. For simplicity, `autonomous_driving.launch` can be used to launch all three nodes within one terminal. For example,
```
roslaunch autonomous_driving.launch no_traffic:=true ego_lane_follower:=true
```
will launch the trivial simulation with no traffic and a lane-following ego vehicle. See `launch/autonomous_driving.launch` for more details. `rviz/config.rviz` is prepared for visualization.

