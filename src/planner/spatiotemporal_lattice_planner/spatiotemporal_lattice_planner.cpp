/*
 * Copyright [2019] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <set>
#include <planner/common/utils.h>
#include <planner/spatiotemporal_lattice_planner/spatiotemporal_lattice_planner.h>

namespace planner {
namespace spatiotemporal_lattice_planner {

constexpr std::array<std::pair<double, double>, 3> Vertex::kSpeedIntervalsPerStation_;
constexpr std::array<double, 6> SpatiotemporalLatticePlanner::kAccelerationOptions_;

const double ConstAccelTrafficSimulator::accelCost(
    const double accel, const double speed, const double policy_speed) const {
  // The cost map for brake.
  // [0, 1) -> 0
  // [1, 2) -> 1
  // [2, 4) -> 2
  // [4, 6) -> 4
  // [6, 8) -> 8
  // [8, +) -> 16
  static std::unordered_map<int, double> cost_map {
    {0, 0.0},
    {1, 1.0},
    {2, 2.0}, {3, 2.0},
    {4, 4.0}, {5, 4.0},
    {6, 8.0}, {7, 8.0},
  };

  // Crusing.
  if (accel == 0.0) return 0.0;

  // Accelerating.
  if (accel > 0.0) {
    if (speed < policy_speed) return -1.0;
    else return 1.0;
  }

  // Braking.
  const int brake_key = static_cast<int>(-accel);
  if (brake_key < 8) return cost_map[brake_key];
  else return 16.0;
}

const double ConstAccelTrafficSimulator::accelCost() const {
  // We consider four vehicles in computing the accel cost.
  // The ego and the followers of the ego vehicle.
  boost::optional<std::pair<size_t, double>> back =
    snapshot_.trafficLattice()->back(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    snapshot_.trafficLattice()->leftBack(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    snapshot_.trafficLattice()->rightBack(snapshot_.ego().id());

  double ego_brake_cost = accelCost(
      snapshot_.ego().acceleration(),
      snapshot_.ego().speed(),
      snapshot_.ego().policySpeed());

  double agent_brake_cost = 0.0;
  // We don't really care if other agents can accelerate or not.
  if (back)
    agent_brake_cost += accelCost(
        snapshot_.vehicle(back->first).acceleration(),
        snapshot_.vehicle(back->first).speed(),
        snapshot_.vehicle(back->first).speed());
  if (left_back)
    agent_brake_cost += accelCost(
        snapshot_.vehicle(left_back->first).acceleration(),
        snapshot_.vehicle(left_back->first).speed(),
        snapshot_.vehicle(left_back->first).speed());
  if (right_back)
    agent_brake_cost += accelCost(
        snapshot_.vehicle(right_back->first).acceleration(),
        snapshot_.vehicle(right_back->first).speed(),
        snapshot_.vehicle(right_back->first).speed());

  return ego_brake_cost + 0.5*agent_brake_cost;
}

void Vertex::updateOptimalParent() {
  // Set the \c optimal_parent_ to an existing parent vertex.
  // It does not matter which parent is used for now.
  if (!optimal_parent_) {
    for (const auto& parent : left_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : back_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : right_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    throw std::runtime_error(
        "Vertex::updateOptimalParent(): "
        "cannot update optimal parent since there is no parent available.\n");
  }

  // Set the \c optimal_parent_ to the existing parent with the minimum cost-to-come.
  // With the same cost-to-come, the back parent is preferred.
  for (const auto& parent : left_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  for (const auto& parent : right_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  for (const auto& parent : back_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  // Update the snapshot at this vertex to the one reached
  // from the optimal parent.
  snapshot_ = std::get<0>(*optimal_parent_);

  return;
}

void Vertex::updateLeftParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  left_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateBackParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  back_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateRightParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  right_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateLeftChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(left_children_[*idx]) ||
      std::get<2>(*(left_children_[*idx])) > stage_cost)
    left_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);

  return;
}

void Vertex::updateFrontChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(front_children_[*idx]) ||
      std::get<2>(*(front_children_[*idx])) > stage_cost)
    front_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);
  return;
}

void Vertex::updateRightChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(right_children_[*idx]) ||
      std::get<2>(*(right_children_[*idx])) > stage_cost)
    right_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);
  return;
}

std::string Vertex::string(const std::string& prefix) const {
  std::string output = prefix;
  output += "node id: " + std::to_string(node_.lock()->id()) + "\n";
  output += "snapshot: \n" + snapshot_.string();

  // Output the parents.
  boost::format parent_format("node id:%1% ego speed:%2% cost to come:%3%\n");

  //std::printf("Get left parents.\n");
  output += std::string("left parents #: ") + std::to_string(leftParentsSize()) + "\n";
  for (const auto& parent : left_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  //std::printf("Get back parents.\n");
  output += std::string("back parents #: ") + std::to_string(backParentsSize()) + "\n";
  for (const auto& parent : back_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  //std::printf("Get right parents.\n");
  output += std::string("right parents #: ") + std::to_string(rightParentsSize()) + "\n";
  for (const auto& parent : right_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  //std::printf("Get the optimal parent.\n");
  output += "optimal parent: ";
  if (optimal_parent_)
    output += (parent_format % std::get<2>(*optimal_parent_).lock()->node().lock()->id()
                             % std::get<2>(*optimal_parent_).lock()->speed()
                             % std::get<1>(*optimal_parent_)).str();
  else output += "\n";

  // Output the children.
  boost::format child_format(
      "node id:%1% ego speed:%2% acceleration:%3% path length:%4% stage cost:%5%\n");

  //std::printf("Get left children.\n");
  output += std::string("left children #: ") + std::to_string(leftChildrenSize()) + "\n";
  for (const auto& child : left_children_) {
    if (!child) continue;
    output += (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  //std::printf("Get front children.\n");
  output += std::string("front children #: ") + std::to_string(frontChildrenSize()) + "\n";
  for (const auto& child : front_children_) {
    if (!child) continue;
    output += (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  //std::printf("Get right children.\n");
  output += std::string("right children #: ") + std::to_string(rightChildrenSize()) + "\n";
  for (const auto& child : right_children_) {
    if (!child) continue;
    output += (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  return output;
}

std::vector<boost::shared_ptr<const WaypointNode>>
  SpatiotemporalLatticePlanner::nodes() const {

  std::set<size_t> visited_nodes;
  std::vector<boost::shared_ptr<const WaypointNode>> nodes_in_graph;

  for (const auto& item : node_to_vertices_table_) {
    for (const auto& vertex : item.second) {
      if (!vertex) continue;
      boost::shared_ptr<const WaypointNode> node = vertex->node().lock();
      if (visited_nodes.count(node->id()) > 0) continue;

      nodes_in_graph.push_back(node);
      visited_nodes.insert(node->id());
    }
  }

  return nodes_in_graph;
}

std::vector<ContinuousPath> SpatiotemporalLatticePlanner::edges() const {
  std::set<size_t> visited_paths;
  std::vector<ContinuousPath> paths_in_graph;

  for (const auto& item : node_to_vertices_table_) {
    for (const auto& vertex : item.second) {
      if (!vertex) continue;
      boost::shared_ptr<const WaypointNode> node = vertex->node().lock();

      // Paths to left children.
      for (const auto& child : vertex->validLeftChildren()) {
        boost::shared_ptr<const WaypointNode> child_node = std::get<3>(child).lock()->node().lock();

        size_t path_id = 0;
        utils::hashCombine(path_id, node->id(), child_node->id());
        if (visited_paths.count(path_id) > 0) continue;

        paths_in_graph.push_back(std::get<0>(child));
        visited_paths.insert(path_id);
      }

      // Paths to front children.
      for (const auto& child : vertex->validFrontChildren()) {
        boost::shared_ptr<const WaypointNode> child_node = std::get<3>(child).lock()->node().lock();

        size_t path_id = 0;
        utils::hashCombine(path_id, node->id(), child_node->id());
        if (visited_paths.count(path_id) > 0) continue;

        paths_in_graph.push_back(std::get<0>(child));
        visited_paths.insert(path_id);
      }

      // Paths to right children.
      for (const auto& child : vertex->validRightChildren()) {
        boost::shared_ptr<const WaypointNode> child_node = std::get<3>(child).lock()->node().lock();

        size_t path_id = 0;
        utils::hashCombine(path_id, node->id(), child_node->id());
        if (visited_paths.count(path_id) > 0) continue;

        paths_in_graph.push_back(std::get<0>(child));
        visited_paths.insert(path_id);
      }
    }
  }

  return paths_in_graph;
}

bool SpatiotemporalLatticePlanner::immediateNextVertexReached(
    const Snapshot& snapshot) const {

  //std::printf("SpatiotemporalLatticePlanner::immediateNextStationReached()\n");

  boost::shared_ptr<const WaypointLattice> waypoint_lattice =
    boost::const_pointer_cast<const WaypointLattice>(waypoint_lattice_);

  // Find out the current distance of the ego on the lattice.
  boost::shared_ptr<const WaypointNode> ego_node = waypoint_lattice->closestNode(
      fast_map_->waypoint(snapshot.ego().transform().location),
      waypoint_lattice->longitudinalResolution());
  const double ego_distance = ego_node->distance();

  // Find out the distance the ego need to achieve.
  const double target_distance = cached_next_vertex_.lock()->node().lock()->distance();

  // If the difference is less than 0.5, or the ego has travelled beyond the
  // the target distance, the immediate station is considered to be reached.
  if (target_distance-ego_distance < 0.5) return true;
  else return false;
}

void SpatiotemporalLatticePlanner::updateWaypointLattice(const Snapshot& snapshot) {

  //std::printf("SpatiotemporalLatticePlanner::updateWaypointLattice()\n");

  // If the waypoint lattice has not been initialized, a new one is created with
  // the start waypoint as where the ego currently is. Meanwhile, the range of
  // the lattice is set to the spatial horizon. The resolution is hardcoded as 1.0m.
  if (!waypoint_lattice_) {
    //std::printf("Create new waypoint lattice.\n");
    boost::shared_ptr<CarlaWaypoint> ego_waypoint =
      fast_map_->waypoint(snapshot.ego().transform().location);
    waypoint_lattice_ = boost::make_shared<WaypointLattice>(
        ego_waypoint, spatial_horizon_+30.0, 1.0, router_);
    return;
  }

  // If the waypoint lattice has been created before, we will choose to either
  // update the lattice or leave it as it currently is based on whether the ego
  // has reached one of the child stations of the root.
  if (immediateNextVertexReached(snapshot)) {
    boost::shared_ptr<const WaypointLattice> waypoint_lattice =
      boost::const_pointer_cast<const WaypointLattice>(waypoint_lattice_);

    boost::shared_ptr<const WaypointNode> ego_node = waypoint_lattice->closestNode(
        fast_map_->waypoint(snapshot.ego().transform().location),
        waypoint_lattice->longitudinalResolution());
    //std::printf("Shift the lattice by %f\n", ego_node->distance()-5.0);
    const double shift_distance = ego_node->distance() - 5.0;
    waypoint_lattice_->shift(shift_distance);
  }

  return;
}

std::list<std::pair<ContinuousPath, double>>
  SpatiotemporalLatticePlanner::planTraj(
    const size_t ego, const Snapshot& snapshot) {

  //std::printf("SpatiotemporalLatticePlanner::planTraj()\n");

  if (ego != snapshot.ego().id()) {
    std::string error_msg(
        "SpatiotemporalLatticePlanner::plan(): "
        "The Spatiotemporal lattice planner can only plan for the ego.\n");
    std::string id_msg = (boost::format(
          "Target vehicle ID:%1% Ego vehicle ID:%2%\n")
        % ego % snapshot.ego().id()).str();

    throw std::runtime_error(error_msg + id_msg);
  }

  // Update the waypoint lattice.
  updateWaypointLattice(snapshot);

  // Prune the vertex graph.
  std::deque<boost::shared_ptr<Vertex>> vertex_queue = pruneVertexGraph(snapshot);

  // Construct the vertex graph.
  constructVertexGraph(vertex_queue);

  // Select the optimal trajectory sequence from the graph.
  std::list<std::pair<ContinuousPath, double>> optimal_traj_seq;
  std::list<boost::weak_ptr<Vertex>> optimal_vertex_seq;
  selectOptimalTraj(optimal_traj_seq, optimal_vertex_seq);

  // Update the cached next vertex.
  cached_next_vertex_ = *(++optimal_vertex_seq.begin());

  //for (const auto& vertex : vertices())
  //  std::printf("%s", vertex->string().c_str());

  return optimal_traj_seq;
}

DiscretePath SpatiotemporalLatticePlanner::planPath(
    const size_t ego, const Snapshot& snapshot) {

  //std::printf("SpatiotemporalLatticePlanner::planPath()\n");

  std::list<std::pair<ContinuousPath, double>> optimal_traj_seq =
    planTraj(ego, snapshot);

  // Merge the path sequence into one discrete path.
  std::list<ContinuousPath> optimal_path_seq;
  std::list<double> optimal_accel_seq;
  for (const auto& traj : optimal_traj_seq) {
    optimal_path_seq.push_back(traj.first);
    optimal_accel_seq.push_back(traj.second);
  }

  DiscretePath optimal_path = mergePaths(optimal_path_seq);
  return optimal_path;
}

std::deque<boost::shared_ptr<Vertex>>
  SpatiotemporalLatticePlanner::pruneVertexGraph(
    const Snapshot& snapshot) {

  //std::printf("SpatiotemporalLatticePlanner::pruneVertexGraph()\n");

  // Stores the vertices to be explored.
  std::deque<boost::shared_ptr<Vertex>> vertex_queue;

  // There are two cases we can basically start fresh in constructing the vertex graph:
  // 1) This is the first time the \c plan() interface is called.
  // 2) The ego reached one of the immediate child of the root vertex.

  if ((!root_.lock()) || immediateNextVertexReached(snapshot)) {
    node_to_vertices_table_.clear();

    // Initialize the new root station.
    boost::shared_ptr<Vertex> root =
      boost::make_shared<Vertex>(snapshot, waypoint_lattice_, fast_map_);
    addVertexToTable(root);
    root_ = root;

    vertex_queue.push_back(root);
    return vertex_queue;
  }

  // If the ego is in the progress of approaching one of the immedidate
  // child of the root node, we have to keep these immediate child nodes
  // where they are.

  // Create the new root station.
  boost::shared_ptr<Vertex> new_root =
    boost::make_shared<Vertex>(snapshot, waypoint_lattice_, fast_map_);

  // Find the immedidate waypoint nodes.
  boost::shared_ptr<const WaypointNode> next_node = cached_next_vertex_.lock()->node().lock();
  const double distance_to_next_node =
    next_node->distance() - new_root->node().lock()->distance();

  boost::shared_ptr<const WaypointNode> front_node =
    waypoint_lattice_->front(new_root->node().lock()->waypoint(), distance_to_next_node);
  boost::shared_ptr<const WaypointNode> left_front_node =
    waypoint_lattice_->frontLeft(new_root->node().lock()->waypoint(), distance_to_next_node);
  boost::shared_ptr<const WaypointNode> right_front_node =
    waypoint_lattice_->frontRight(new_root->node().lock()->waypoint(), distance_to_next_node);

  // Clear all old vertices.
  // We are good with the above nodes already. All vertices will be newly created.
  node_to_vertices_table_.clear();

  // Try to connect the new root with above nodes.
  std::vector<boost::shared_ptr<Vertex>> front_vertices =
    connectVertexToFrontNode(new_root, front_node);
  std::vector<boost::shared_ptr<Vertex>> left_front_vertices =
    connectVertexToLeftFrontNode(new_root, left_front_node);
  std::vector<boost::shared_ptr<Vertex>> right_front_vertices =
    connectVertexToRightFrontNode(new_root, right_front_node);

  // Save the new root to the table.
  root_ = new_root;
  addVertexToTable(new_root);

  // Save the newly created vertices to the table and queue
  // if they are successfully created.
  for (const auto& vertex : front_vertices) {
    addVertexToTable(vertex);
    if (vertex->node().lock()->id() == front_node->id())
      vertex_queue.push_back(vertex);
  }

  for (const auto& vertex : left_front_vertices) {
    addVertexToTable(vertex);
    if (vertex->node().lock()->id() == left_front_node->id())
      vertex_queue.push_back(vertex);
  }

  for (const auto& vertex : right_front_vertices) {
    addVertexToTable(vertex);
    if (vertex->node().lock()->id() == right_front_node->id())
      vertex_queue.push_back(vertex);
  }

  return vertex_queue;
}

void SpatiotemporalLatticePlanner::constructVertexGraph(
    std::deque<boost::shared_ptr<Vertex>>& vertex_queue) {

  //std::printf("SpatiotemporalLatticePlanner::constructVertexGraph()\n");

  auto addVerticesToTableAndQueue = [this, &vertex_queue](
      const std::vector<boost::shared_ptr<Vertex>>& vertices,
      const boost::shared_ptr<const WaypointNode>& node)->void{

    if (vertices.size()==0 || (!node)) return;

    // Try to add the new vertices to the table and queue.
    for (const auto& vertex : vertices) {
      // The vertex already exists in the table.
      // Therefore, we don't have to add it to the table or the queue.
      if(findVertexInTable(vertex)) continue;

      // If the vertex reaches the target node,
      // add the vertex to the queue and table in order to expand later.
      addVertexToTable(vertex);
      if (vertex->node().lock()->id() == node->id())
        vertex_queue.push_back(vertex);
    }
  };

  while (!vertex_queue.empty()) {
    // Get the next vertex to expand.
    boost::shared_ptr<Vertex> vertex = vertex_queue.front();
    vertex_queue.pop_front();

    // Try to connect to the front node.
    boost::shared_ptr<const WaypointNode> front_node =
      waypoint_lattice_->front(vertex->node().lock()->waypoint(), 50.0);
    std::vector<boost::shared_ptr<Vertex>> front_vertices =
      connectVertexToFrontNode(vertex, front_node);

    addVerticesToTableAndQueue(front_vertices, front_node);

    // Try to connect to the left front node.
    boost::shared_ptr<const WaypointNode> left_front_node =
      waypoint_lattice_->leftFront(vertex->node().lock()->waypoint(), 50.0);
    std::vector<boost::shared_ptr<Vertex>> left_front_vertices =
      connectVertexToLeftFrontNode(vertex, left_front_node);

    addVerticesToTableAndQueue(left_front_vertices, left_front_node);

    // Try to connect to the right front node.
    boost::shared_ptr<const WaypointNode> right_front_node =
      waypoint_lattice_->rightFront(vertex->node().lock()->waypoint(), 50.0);
    std::vector<boost::shared_ptr<Vertex>> right_front_vertices =
      connectVertexToRightFrontNode(vertex, right_front_node);

    addVerticesToTableAndQueue(right_front_vertices, right_front_node);
  }

  //std::printf("===========================================\n");
  //for (const auto& vertex : this->vertices())
  //  std::printf("%s\n", vertex->string().c_str());
  //std::cin.get();
  //std::printf("===========================================\n");

  return;
}

std::vector<boost::shared_ptr<Vertex>>
  SpatiotemporalLatticePlanner::connectVertexToFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("SpatiotemporalLatticePlanner::connectVertexToFrontNode()\n");

  // Stores the expanded front children of the input vertex.
  std::array<boost::shared_ptr<Vertex>, Vertex::kSpeedIntervalsPerStation_.size()>
    front_children;

  // Return directly if the target node does not exist.
  if (!target_node) return std::vector<boost::shared_ptr<Vertex>>();

  // Plan a path between the node at the current vertex to the target node.
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::KeepLane);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created, the front vertices
    // cannot be created either.
    std::printf("%s", e.what());
    return std::vector<boost::shared_ptr<Vertex>>();
  }

  // Simulate the traffic forward with the ego applying different constant
  // accelerations over the path created above.
  for (const double accel : kAccelerationOptions_) {
    // Prepare the start snapshot.
    // The acceleration of the ego is set accordingly.
    Snapshot snapshot = vertex->snapshot();
    snapshot.ego().acceleration() = accel;

    ConstAccelTrafficSimulator simulator(snapshot, map_, fast_map_);
    double simulation_time = 0.0; double stage_cost = 0.0;
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);

    // Continue if this acceleration option leads to collision.
    if (!no_collision) continue;

    // Create a new vertex using the end snapshot of the simulation.
    boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
        simulator.snapshot(), waypoint_lattice_, fast_map_);

    // Check if a similar vertex (close in ego velocity) has already been created.
    // If so, the \c next_vertex is replaced with the existing one in the table.
    boost::shared_ptr<Vertex> similar_vertex = findVertexInTable(next_vertex);
    if (similar_vertex) next_vertex = similar_vertex;

    //// Ignore this option if the target node is not reached under this
    //// acceleration option.
    //if (next_vertex->node().lock()->id() != target_node->id())
    //  continue;

    // Update the child of the parent vertex.
    vertex->updateFrontChild(*path, accel, stage_cost, next_vertex);

    // Update the parent vertex of the child.
    if (vertex->hasParents()) {
      next_vertex->updateBackParent(
          simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);
    } else {
      next_vertex->updateBackParent(
          simulator.snapshot(), stage_cost, vertex);
    }

    //std::printf("----------------------------------------------\n");
    //std::printf("acceleration option: %f\n", accel);
    //std::printf("%s", vertex->string("start vertex:\n").c_str());
    //std::printf("%s", next_vertex->string("new vertex:\n").c_str());
    //std::cin.get();
    //std::printf("----------------------------------------------\n");

    // Set the front vertices that are connected with this vertex.
    auto children = vertex->frontChildren();
    for (size_t i = 0; i < Vertex::kSpeedIntervalsPerStation_.size(); ++i) {
      if (!(children[i])) continue;
      front_children[i] = std::get<3>(*(children[i])).lock();
    }

  } // End for loop for different acceleration options.

  // Collect all the front child vertices of the input vertex.
  std::vector<boost::shared_ptr<Vertex>> output_vertices;
  for (const auto& child : front_children) {
    if (!child) continue;
    output_vertices.push_back(child);
  }

  return output_vertices;
}

std::vector<boost::shared_ptr<Vertex>>
  SpatiotemporalLatticePlanner::connectVertexToLeftFrontNode(
    const boost::shared_ptr<Vertex>& vertex,
    const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("SpatiotemporalLatticePlanner::connectVertexToLeftFrontNode()\n");

  // Stores the expanded left front children of the input vertex.
  std::array<boost::shared_ptr<Vertex>, Vertex::kSpeedIntervalsPerStation_.size()>
    left_children;

  // Return directly if the target node does not exist.
  if (!target_node) return std::vector<boost::shared_ptr<Vertex>>();

  // Return directly if the target node is already very close to the vertex.
  // It is not reasonable to change lane with this short distance.
  if (target_node->distance()-vertex->node().lock()->distance() < 20.0)
    return std::vector<boost::shared_ptr<Vertex>>();

  // Check the left front and left back vehicles.
  //
  // If there are vehicles at the left front or left back of the ego,
  // meanwhile those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> left_front =
    vertex->snapshot().trafficLattice()->leftFront(vertex->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    vertex->snapshot().trafficLattice()->leftBack(vertex->snapshot().ego().id());

  if (left_front && left_front->second <= 0.0)
    return std::vector<boost::shared_ptr<Vertex>>();
  if (left_back && left_back->second <= 0.0)
    return std::vector<boost::shared_ptr<Vertex>>();

  // Plan a path between the node at the current vertex to the target node.
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::LeftLaneChange);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created, the front vertices
    // cannot be created either.
    std::printf("%s", e.what());
    return std::vector<boost::shared_ptr<Vertex>>();
  }

  // Simulate the traffic forward with the ego applying different constant
  // accelerations over the path created above.
  for (const double accel : kAccelerationOptions_) {
    // Prepare the start snapshot.
    // The acceleration of the ego is set accordingly.
    Snapshot snapshot = vertex->snapshot();
    snapshot.ego().acceleration() = accel;

    ConstAccelTrafficSimulator simulator(snapshot, map_, fast_map_);
    double simulation_time = 0.0; double stage_cost = 0.0;
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);

    // Continue if this acceleration option leads to collision.
    if (!no_collision) continue;

    // Create a new vertex using the end snapshot of the simulation.
    boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
        simulator.snapshot(), waypoint_lattice_, fast_map_);

    // Check if a similar vertex (close in ego velocity) has already been created.
    // If so, the \c next_vertex is replaced with the existing one in the table.
    boost::shared_ptr<Vertex> similar_vertex = findVertexInTable(next_vertex);
    if (similar_vertex) next_vertex = similar_vertex;

    //// Ignore this option if the target node is not reached under this
    //// acceleration option.
    //if (next_vertex->node().lock()->id() != target_node->id())
    //  continue;

    // Update the child of the parent vertex.
    vertex->updateLeftChild(*path, accel, stage_cost, next_vertex);

    // Update the parent vertex of the child.
    if (vertex->hasParents()) {
      next_vertex->updateRightParent(
          simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);
    } else {
      next_vertex->updateRightParent(
          simulator.snapshot(), stage_cost, vertex);
    }

    // Set the left front vertices that are connected with this vertex.
    auto children = vertex->leftChildren();
    for (size_t i = 0; i < Vertex::kSpeedIntervalsPerStation_.size(); ++i) {
      if (!(children[i])) continue;
      left_children[i] = std::get<3>(*(children[i])).lock();
    }
  } // End for loop for different acceleration options.

  // Collect all the left child vertices of the input vertex.
  std::vector<boost::shared_ptr<Vertex>> output_vertices;
  for (const auto& child : left_children) {
    if (!child) continue;
    output_vertices.push_back(child);
  }

  return output_vertices;
}

std::vector<boost::shared_ptr<Vertex>>
  SpatiotemporalLatticePlanner::connectVertexToRightFrontNode(
    const boost::shared_ptr<Vertex>& vertex,
    const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("SpatiotemporalLatticePlanner::connectVertexToRightFrontNode()\n");

  // Stores the expanded right front children of the input vertex.
  std::array<boost::shared_ptr<Vertex>, Vertex::kSpeedIntervalsPerStation_.size()>
    right_children;

  // Return directly if the target node does not exist.
  if (!target_node) return std::vector<boost::shared_ptr<Vertex>>();

  // Return directly if the target node is already very close to the vertex.
  // It is not reasonable to change lane with this short distance.
  if (target_node->distance()-vertex->node().lock()->distance() < 20.0)
    return std::vector<boost::shared_ptr<Vertex>>();

  // Check the right front and right back vehicles.
  //
  // If there are vehicles at the right front or right back of the ego,
  // meanwhile those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> right_front =
    vertex->snapshot().trafficLattice()->rightFront(vertex->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    vertex->snapshot().trafficLattice()->rightBack(vertex->snapshot().ego().id());

  if (right_front && right_front->second <= 0.0)
    return std::vector<boost::shared_ptr<Vertex>>();
  if (right_back && right_back->second <= 0.0)
    return std::vector<boost::shared_ptr<Vertex>>();

  // Plan a path between the node at the current vertex to the target node.
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::RightLaneChange);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created, the front vertices
    // cannot be created either.
    std::printf("%s", e.what());
    return std::vector<boost::shared_ptr<Vertex>>();
  }

  // Simulate the traffic forward with the ego applying different constant
  // accelerations over the path created above.
  for (const double accel : kAccelerationOptions_) {
    // Prepare the start snapshot.
    // The acceleration of the ego is set accordingly.
    Snapshot snapshot = vertex->snapshot();
    snapshot.ego().acceleration() = accel;

    ConstAccelTrafficSimulator simulator(snapshot, map_, fast_map_);
    double simulation_time = 0.0; double stage_cost = 0.0;
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);

    // Continue if this acceleration option leads to collision.
    if (!no_collision) continue;

    // Create a new vertex using the end snapshot of the simulation.
    boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
        simulator.snapshot(), waypoint_lattice_, fast_map_);

    // Check if a similar vertex (close in ego velocity) has already been created.
    // If so, the \c next_vertex is replaced with the existing one in the table.
    boost::shared_ptr<Vertex> similar_vertex = findVertexInTable(next_vertex);
    if (similar_vertex) next_vertex = similar_vertex;

    //// Ignore this option if the target node is not reached under this
    //// acceleration option.
    //if (next_vertex->node().lock()->id() != target_node->id())
    //  continue;

    // Update the child of the parent vertex.
    vertex->updateRightChild(*path, accel, stage_cost, next_vertex);

    // Update the parent vertex of the child.
    if (vertex->hasParents()) {
      next_vertex->updateLeftParent(
          simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);
    } else {
      next_vertex->updateLeftParent(
          simulator.snapshot(), stage_cost, vertex);
    }

    // Set the right front vertices that are connected with this vertex.
    auto children = vertex->rightChildren();
    for (size_t i = 0; i < Vertex::kSpeedIntervalsPerStation_.size(); ++i) {
      if (!(children[i])) continue;
      right_children[i] = std::get<3>(*(children[i])).lock();
    }
  } // End for loop for different acceleration options.

  // Collect all the right child vertices of the input vertex.
  std::vector<boost::shared_ptr<Vertex>> output_vertices;
  for (const auto& child : right_children) {
    if (!child) continue;
    output_vertices.push_back(child);
  }

  return output_vertices;
}

boost::shared_ptr<Vertex> SpatiotemporalLatticePlanner::findVertexInTable(
    const boost::shared_ptr<Vertex>& vertex) {

  //std::printf("SpatiotemporalLatticePlanner::findVertexInTable()\n");

  boost::optional<size_t> idx = Vertex::speedIntervalIdx(vertex->speed());
  if (!idx) {
    std::string error_msg(
        "SpatiotemporalLattice::findVertexInTable(): "
        "invalid ego speed in input vertex.\n");
    error_msg + vertex->string();
    throw std::runtime_error(error_msg);
  }

  auto iter = node_to_vertices_table_.find(vertex->node().lock()->id());
  if (iter == node_to_vertices_table_.end()) return nullptr;
  return (iter->second)[*idx];
}

const double SpatiotemporalLatticePlanner::terminalSpeedCost(
    const boost::shared_ptr<Vertex>& vertex) const {

  if (vertex->hasChildren()) {
    std::string error_msg(
        "SpatiotemporalLatticePlanner::terminalSpeedCost(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + vertex->string());
  }

  static std::unordered_map<int, double> cost_map {
    {0, 4.0}, {1, 4.0}, {2, 4.0}, {3, 3.0}, {4, 3.0},
    {5, 2.0}, {6, 2.0}, {7, 1.0}, {8, 1.0}, {9, 0.0},
  };

  const double ego_speed = vertex->speed();
  const double ego_policy_speed = vertex->snapshot().ego().policySpeed();
  if (ego_speed < 0.0 || ego_policy_speed < 0.0) {
    std::string error_msg(
        "SpatiotemporalLatticePlanner::terminalSpeedCost(): "
        "ego speed<0.0 or ego policy speed<0.0.\n");
    std::string speed_msg = (boost::format(
          "ego speed:%1% ego policy speed:%2%\n")
          % ego_speed
          % ego_policy_speed).str();
    throw std::runtime_error(error_msg + speed_msg);
  }

  // FIXME: What if the policy speed is 0.
  //        Should not worry about this too much here since,
  //        1) IDM does not support 0 policy speed.
  //        2) 0 policy speed may cause some invalidity in carla, such as
  //           looking for a waypoint 0m ahead.
  const double speed_ratio = ego_speed / ego_policy_speed;

  // There is no cost if the speed of the ego matches or exceeds the policy speed.
  if (speed_ratio >= 1.0) return 0.0;
  else return cost_map[static_cast<int>(speed_ratio*10.0)];
}

const double SpatiotemporalLatticePlanner::terminalDistanceCost(
    const boost::shared_ptr<Vertex>& vertex) const {

  if (vertex->hasChildren()) {
    std::string error_msg(
        "SpatiotemporalLatticePlanner::terminalSpeedCost(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + vertex->string());
  }

  static std::unordered_map<int, double> cost_map {
    {0, 20.0}, {1, 20.0}, {2, 20.0}, {3, 20.0}, {4, 20.0},
    {5, 20.0}, {6, 20.0}, {7, 20.0}, {8, 10.0},  {9, 5.0},
  };

  // Find the current spatial planning horizon.
  boost::shared_ptr<const Vertex> root_child;
  if (root_.lock()->hasFrontChildren())
    root_child = std::get<3>(root_.lock()->validFrontChildren().front()).lock();
  else if (root_.lock()->hasLeftChildren())
    root_child = std::get<3>(root_.lock()->validLeftChildren().front()).lock();
  else if (root_.lock()->hasRightChildren())
    root_child = std::get<3>(root_.lock()->validRightChildren().front()).lock();

  const double spatial_horizon =
    spatial_horizon_ - 50.0 +
    root_child->node()->distance() -
    root_.lock()->node().lock()->distance();

  const double distance = vertex->node().lock()->distance() -
                          root_.lock()->node().lock()->distance();
  const double distance_ratio = distance / spatial_horizon_;

  if (distance_ratio >= 1.0) return 0.0;
  else return cost_map[static_cast<int>(distance_ratio*10.0)];
}

const double SpatiotemporalLatticePlanner::costFromRootToTerminal(
    const boost::shared_ptr<Vertex>& terminal) const {

  if (terminal->hasChildren()) {
    std::string error_msg(
        "SpatiotemporalLatticePlanner::terminalSpeedCost(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + terminal->string());
  }

  const double path_cost = terminal->costToCome();
  const double terminal_speed_cost = terminalSpeedCost(terminal);
  const double terminal_distance_cost = terminalDistanceCost(terminal);

  // TODO: Weight the cost properly.
  return path_cost + terminal_speed_cost + terminal_distance_cost;
}

void SpatiotemporalLatticePlanner::selectOptimalTraj(
    std::list<std::pair<ContinuousPath, double>>& traj_sequence,
    std::list<boost::weak_ptr<Vertex>>& vertex_sequence) const {

  //std::printf("SpatiotemporalLatticePlanner::selectOptimalTraj()\n");

  //std::printf("Find optimal terminal station.\n");
  boost::shared_ptr<Vertex> optimal_vertex = nullptr;
  // Set the initial cost to a large enough number.
  double optimal_cost = 1.0e10;

  // Find the optimal terminal vertex.
  for (const auto& item : node_to_vertices_table_) {
    for (const auto& vertex : item.second) {

      if (!vertex) continue;
      // Only terminal stations are considered, i.e. stations without children.
      if (vertex->hasChildren()) continue;

      const double vertex_cost = costFromRootToTerminal(vertex);

      // Update the optimal station if the candidate has small cost.
      if (vertex_cost < optimal_cost) {
        optimal_vertex = vertex;
        optimal_cost = vertex_cost;
      }
    }
  }

  // The optimal vertex should be set no matter what.
  if (!optimal_vertex) {
    throw std::runtime_error(
        "SpatiotemporalLatticePlanner::selectOptimalTraj(): "
        "no terminal vertex in the graph.\n");
  }

  // There should always be parent vertices for a terminal vertex.
  // Otherwise, there is just one root vertex in the graph.
  if (!optimal_vertex->hasParents()) {
    throw std::runtime_error(
        "SpatiotemporalLatticePlanner::selectOptimalTraj(): "
        "the graph only has root vertex.\n");
  }

  // Trace back from the terminal vertex to find all the trajectories
  // (path + acceleration).
  traj_sequence.clear();
  vertex_sequence.clear();

  boost::shared_ptr<Vertex> vertex = optimal_vertex;
  vertex_sequence.push_front(boost::weak_ptr<Vertex>(vertex));

  while (vertex->hasParents()) {
    //std::printf("%s", vertex->string().c_str());

    // Find the parent vertex of this one.
    boost::shared_ptr<Vertex> parent_vertex =
      std::get<2>((*(vertex->optimalParent()))).lock();
    if (!parent_vertex) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::selectOptimalTraj(): "
          "cannot find parent when tracing back optimal trajectory from the vertex.\n");
      throw std::runtime_error(error_msg + vertex->string());
    }

    vertex_sequence.push_front(boost::weak_ptr<Vertex>(parent_vertex));

    // Find the path between the parent and this vertex.
    boost::optional<std::pair<ContinuousPath, double>> traj =
      findTrajFromParentToChild(parent_vertex, vertex);

    if (!traj) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::selectOptimalTraj(): "
          "vertex is not a child of parent.\n");
      error_msg += vertex->string("vertex: \n") +
                   parent_vertex->string("parent vertex: \n");
      throw std::runtime_error(error_msg);
    }

    traj_sequence.push_front(*traj);
    vertex = parent_vertex;
  }
  //std::printf("%s", vertex->string().c_str());

  return;
}

DiscretePath SpatiotemporalLatticePlanner::mergePaths(
    const std::list<ContinuousPath>& paths) const {
  //std::printf("SpatiotemporalLatticePlanner::mergePaths()\n");
  DiscretePath path(paths.front());
  for (std::list<ContinuousPath>::const_iterator iter = ++(paths.begin());
       iter != paths.end(); ++iter) path.append(*iter);
  return path;
}

boost::optional<std::pair<ContinuousPath, double>>
  SpatiotemporalLatticePlanner::findTrajFromParentToChild(
    const boost::shared_ptr<Vertex>& parent,
    const boost::shared_ptr<Vertex>& child) const {

  //std::printf("SpatiotemporalLatticePlanner::findTrajfromParentToChild()\n");

  // FIXME: The three blocks of code are very similar.
  //        Maybe consider merging them into one function call.

  // Check if the input child is a left child.
  const auto& left_children = parent->leftChildren();

  for (const auto& candidate : left_children) {
    if (!candidate) continue;
    // Stop if the left children does not share the same node with the input child.
    boost::shared_ptr<const Vertex> candidate_vertex = std::get<3>(*candidate).lock();
    if (candidate_vertex->node()->id() != child->node().lock()->id()) continue;

    // Figure out the which child the input child actually is.
    boost::optional<size_t> idx = Vertex::speedIntervalIdx(child->speed());
    if (!idx) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::findTrajFromParentToChild(): "
          "The desired left child is missing.\n");
      error_msg += parent->string("parent:\n") + child->string("child:\n");
      throw std::runtime_error(error_msg);
    }

    return std::make_pair(std::get<0>(*(left_children[*idx])),
                          std::get<1>(*(left_children[*idx])));
  }

  // Check if the input child is a front child.
  const auto& front_children = parent->frontChildren();

  for (const auto& candidate : front_children) {
    if (!candidate) continue;
    // Stop if the left children does not share the same node with the input child.
    boost::shared_ptr<const Vertex> candidate_vertex = std::get<3>(*candidate).lock();
    if (candidate_vertex->node()->id() != child->node().lock()->id()) continue;

    // Figure out the which child the input child actually is.
    boost::optional<size_t> idx = Vertex::speedIntervalIdx(child->speed());
    if (!idx) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::findTrajFromParentToChild(): "
          "The desired front child is missing.\n");
      error_msg += parent->string("parent:\n") + child->string("child:\n");
      throw std::runtime_error(error_msg);
    }

    return std::make_pair(std::get<0>(*(front_children[*idx])),
                          std::get<1>(*(front_children[*idx])));
  }

  // Check if the input child is a right child.
  const auto& right_children = parent->rightChildren();

  for (const auto& candidate : right_children) {
    if (!candidate) continue;
    // Stop if the left children does not share the same node with the input child.
    boost::shared_ptr<const Vertex> candidate_vertex = std::get<3>(*candidate).lock();
    if (candidate_vertex->node()->id() != child->node().lock()->id()) continue;

    // Figure out the which child the input child actually is.
    boost::optional<size_t> idx = Vertex::speedIntervalIdx(child->speed());
    if (!idx) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::findTrajFromParentToChild(): "
          "The desired right child is missing.\n");
      error_msg += parent->string("parent:\n") + child->string("child:\n");
      throw std::runtime_error(error_msg);
    }

    return std::make_pair(std::get<0>(*(right_children[*idx])),
                          std::get<1>(*(right_children[*idx])));
  }

  // If the \c child vertex is not found, return \c boost::none.
  return boost::none;
}

} // End namespace spatiotemporal_lattice_planner.
} // End namespace planner.

