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
#include <list>
#include <planner/common/utils.h>
#include <planner/slc_lattice_planner/slc_lattice_planner.h>

namespace planner {
namespace slc_lattice_planner {

const double SLCTrafficSimulator::egoAcceleration() const {

  // The logic for computing the acceleration for the ego vehicle is simple.
  // Regardless whether the ego vehicle is in the process of lane changing
  // or not, we will only consider the front vehicle (if any) on the same
  // lane with the head of the ego vehicle.
  //
  // TODO: Should we consider the front vehicle on two lanes, both the
  //       old lane and the target lane?

  double accel = 0.0;
  boost::optional<std::pair<size_t, double>> lead =
    snapshot_.trafficLattice()->front(snapshot_.ego().id());

  if (lead) {
    const double lead_speed = snapshot_.vehicle(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.ego().speed(),
                      snapshot_.ego().policySpeed(),
                      lead_speed,
                      following_distance);
  } else {
    accel = idm_->idm(snapshot_.ego().speed(),
                      snapshot_.ego().policySpeed());
  }

  return accel;
}

const double SLCTrafficSimulator::agentAcceleration(const size_t agent) const {

  // We assume all agent vehicles are lane followers for now.
  double accel = 0.0;
  boost::optional<std::pair<size_t, double>> lead =
    snapshot_.trafficLattice()->front(agent);

  if (lead) {
    const double lead_speed = snapshot_.vehicle(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.vehicle(agent).speed(),
                      snapshot_.vehicle(agent).policySpeed(),
                      lead_speed,
                      following_distance);
  } else {
    accel = idm_->idm(snapshot_.vehicle(agent).speed(),
                      snapshot_.vehicle(agent).policySpeed());
  }

  return accel;
}

void Vertex::updateParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  parent_ = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  snapshot_ = snapshot;
  return;
}

void Vertex::updateLeftChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  left_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

void Vertex::updateFrontChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  front_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

void Vertex::updateRightChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  right_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

const bool Vertex::sameLaneWith(
    const boost::shared_ptr<const Vertex>& other) const {

  if (!other)
    throw std::runtime_error("Vertex::sameLaneWith(): other==nullptr\n");

  boost::shared_ptr<const WaypointNode> this_node = node_.lock();
  boost::shared_ptr<const WaypointNode> other_node = other->node();
  boost::shared_ptr<const WaypointNode> node;

  // Search backwards.
  node = this_node;
  while (node) {
    if (node->id() == other_node->id()) return true;
    if (node->left() && node->left()->id()==other_node->id()) return false;
    if (node->right() && node->right()->id()==other_node->id()) return false;
    node = node->back();
  }

  // Search forward.
  node = this_node;
  while (node) {
    if (node->id() == other_node->id()) return true;
    if (node->left() && node->left()->id()==other_node->id()) return false;
    if (node->right() && node->right()->id()==other_node->id()) return false;
    node = node->front();
  }

  return false;
}

std::string Vertex::string(const std::string& prefix) const {
  std::string output = prefix;
  output += "node id: " + std::to_string(node_.lock()->id()) + "\n";
  output += "snapshot: \n" + snapshot_.string();

  boost::format parent_format("node id:%1% cost to come:%2%\n");
  output += "parent: ";
  if (parent_)
    output += (parent_format % std::get<2>(*parent_).lock()->node().lock()->id()
                             % std::get<1>(*parent_)).str();
  else output += "\n";

  boost::format child_format("node id:%1% path length:%2% stage cost:%3%\n");

  output += "front child: ";
  if (front_child_)
    output += (child_format % std::get<2>(*front_child_).lock()->node().lock()->id()
                            % std::get<0>(*front_child_).range()
                            % std::get<1>(*front_child_)).str();
  else output += "\n";

  output += "left child: ";
  if (left_child_)
    output += (child_format % std::get<2>(*left_child_).lock()->node().lock()->id()
                            % std::get<0>(*left_child_).range()
                            % std::get<1>(*left_child_)).str();
  else output += "\n";

  output += "right child: ";
  if (right_child_)
    output += (child_format % std::get<2>(*right_child_).lock()->node().lock()->id()
                            % std::get<0>(*right_child_).range()
                            % std::get<1>(*right_child_)).str();
  else output += "\n";

  return output;
}

std::vector<boost::shared_ptr<const WaypointNode>> SLCLatticePlanner::nodes() const {
  // Different vertices can be at the same waypoint node on the lattice.
  // The function only return nonrepeated waypoint nodes.
  std::set<size_t> visited_nodes;
  std::vector<boost::shared_ptr<const WaypointNode>> nodes_in_graph;

  for (const auto& vertex : all_vertices_) {
    boost::shared_ptr<const WaypointNode> node = vertex->node().lock();
    if (visited_nodes.count(node->id()) > 0) continue;

    nodes_in_graph.push_back(node);
    visited_nodes.insert(node->id());
  }

  return nodes_in_graph;
}

std::vector<ContinuousPath> SLCLatticePlanner::edges() const {
  // Two different pair of vertices may share the same path since they are
  // at the same pair of waypoint nodes. This function only returns the
  // nonrepeated paths.
  std::set<size_t> visited_paths;
  std::vector<ContinuousPath> paths_in_graph;

  auto insertPath = [&visited_paths, &paths_in_graph](
      const boost::shared_ptr<const WaypointNode>& parent_node,
      const boost::shared_ptr<const WaypointNode>& child_node,
      const ContinuousPath& path)->void{
    size_t path_id = 0;
    utils::hashCombine(path_id, parent_node->id(), child_node->id());

    if (visited_paths.count(path_id) > 0) return;
    paths_in_graph.push_back(path);
    visited_paths.insert(path_id);
  };

  for (const auto& vertex : all_vertices_) {

    // Add the path to the front child vertex.
    if (vertex->hasFrontChild()) {
      boost::shared_ptr<const WaypointNode> node = vertex->node().lock();
      boost::shared_ptr<const WaypointNode> child_node =
        std::get<2>(*(vertex->frontChild())).lock()->node().lock();
      const ContinuousPath& path = std::get<0>(*(vertex->frontChild()));
      insertPath(node, child_node, path);
    }

    // Add the path to the left child vertex.
    if (vertex->hasLeftChild()) {
      boost::shared_ptr<const WaypointNode> node = vertex->node().lock();
      boost::shared_ptr<const WaypointNode> child_node =
        std::get<2>(*(vertex->leftChild())).lock()->node().lock();
      const ContinuousPath& path = std::get<0>(*(vertex->leftChild()));
      insertPath(node, child_node, path);
    }

    // Add the path to the right child vertex.
    if (vertex->hasRightChild()) {
      boost::shared_ptr<const WaypointNode> node = vertex->node().lock();
      boost::shared_ptr<const WaypointNode> child_node =
        std::get<2>(*(vertex->rightChild())).lock()->node().lock();
      const ContinuousPath& path = std::get<0>(*(vertex->rightChild()));
      insertPath(node, child_node, path);
    }
  }

  return paths_in_graph;
}

DiscretePath SLCLatticePlanner::planPath(
    const size_t ego, const Snapshot& snapshot) {

  if (ego != snapshot.ego().id()) {
    std::string error_msg(
        "SLCLatticePlanner::planPath(): "
        "The SLC lattice planner can only plan for the ego.\n");
    std::string id_msg = (boost::format(
          "Target vehicle ID:%1% Ego vehicle ID:%2%\n")
        % ego % snapshot.ego().id()).str();

    throw std::runtime_error(error_msg + id_msg);
  }

  // Update the waypoint lattice.
  updateWaypointLattice(snapshot);

  // Prune the vertex graph from the last planning step.
  std::deque<boost::shared_ptr<Vertex>> vertex_queue = pruneVertexGraph(snapshot);

  // In the case that no immedinate front nodes can be connected.
  // We have to start fresh.
  if (vertex_queue.size() == 0) {
    std::string warning_msg;
    warning_msg += snapshot.string("Input snapshot:\n");
    std::printf("SLCLatticePlanner::planPath(): WARNING\n"
                "The ego cannot reach any immediate next nodes. Restart fresh.\n"
                "%s", warning_msg.c_str());

    waypoint_lattice_ = nullptr;
    all_vertices_.clear();

    updateWaypointLattice(snapshot);
    vertex_queue = pruneVertexGraph(snapshot);
  }

  // Construct the vertex graph.
  constructVertexGraph(vertex_queue);

  // Select the optimal path sequence from the vertex graph.
  std::list<ContinuousPath> optimal_path_seq;
  std::list<boost::weak_ptr<Vertex>> optimal_vertex_seq;
  selectOptimalPath(optimal_path_seq, optimal_vertex_seq);

  // Merge the path sequence into one discrete path.
  DiscretePath optimal_path = mergePaths(optimal_path_seq);

  // Update the cached next vertex.
  cached_next_vertex_ = *(++optimal_vertex_seq.begin());

  return optimal_path;
}

bool SLCLatticePlanner::immediateNextVertexReached(
    const Snapshot& snapshot) const {

  //std::printf("immediateNextVertexReached()\n");

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
  // the target distance, the immediate vertex is considered to be reached.
  if (target_distance-ego_distance < 0.5) return true;
  else return false;
}

void SLCLatticePlanner::updateWaypointLattice(const Snapshot& snapshot) {

  //std::printf("updateWaypointLattice(): \n");

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
  // has reached one of the child vertices of the root.
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

std::deque<boost::shared_ptr<Vertex>>
  SLCLatticePlanner::pruneVertexGraph(const Snapshot& snapshot) {

  //std::printf("pruneVertexGraph(): \n");

  // Stores the vertices to be explored.
  std::deque<boost::shared_ptr<Vertex>> vertex_queue;

  // There are two cases we can basically start fresh in constructing the vertex graph:
  // 1) This is the first time the \c plan() interface is called.
  // 2) The ego reached one of the immediate child of the root vertex.
  if ((!root_.lock()) || immediateNextVertexReached(snapshot)) {
    all_vertices_.clear();

    // Initialize the new root vertex.
    boost::shared_ptr<Vertex> root =
      boost::make_shared<Vertex>(snapshot, waypoint_lattice_, fast_map_);
    all_vertices_.push_back(root);
    root_ = root;

    vertex_queue.push_back(root);
    return vertex_queue;
  }

  // If the ego is in the progress of approaching one of the immedidate
  // child of the root node, we have to keep these immediate child nodes
  // where they are.

  // Create the new root vertex.
  boost::shared_ptr<Vertex> new_root =
    boost::make_shared<Vertex>(snapshot, waypoint_lattice_, fast_map_);

  // Read the immedinate next waypoint node to be reached.
  boost::shared_ptr<const WaypointNode> next_node =
    cached_next_vertex_.lock()->node().lock();
  const double distance_to_next_node =
    next_node->distance() - new_root->node().lock()->distance();

  // Find the next nodes to be reached.
  boost::shared_ptr<const WaypointNode> front_node =
    waypoint_lattice_->front(new_root->node().lock()->waypoint(), distance_to_next_node);
  boost::shared_ptr<const WaypointNode> left_front_node =
    waypoint_lattice_->frontLeft(new_root->node().lock()->waypoint(), distance_to_next_node);
  boost::shared_ptr<const WaypointNode> right_front_node =
    waypoint_lattice_->frontRight(new_root->node().lock()->waypoint(), distance_to_next_node);

  // Clear all old vertices.
  // We are good with the previously created nodes. All vertices will be newly created.
  all_vertices_.clear();

  // Try to connect the new root with above nodes.
  boost::shared_ptr<Vertex> front_vertex =
    connectVertexToFrontNode(new_root, front_node);
  boost::shared_ptr<Vertex> left_front_vertex =
    connectVertexToLeftFrontNode(new_root, left_front_node);
  boost::shared_ptr<Vertex> right_front_vertex =
    connectVertexToRightFrontNode(new_root, right_front_node);

  // Save the new root to the graph.
  root_ = new_root;
  all_vertices_.push_back(new_root);

  // Save the newly created vertices to the graph and queue
  // if they are successfully created.
  if (front_vertex) {
    all_vertices_.push_back(front_vertex);
    if (front_vertex->node().lock()->id() == front_node->id())
      vertex_queue.push_back(front_vertex);
  }

  if (left_front_vertex) {
    all_vertices_.push_back(left_front_vertex);
    if (left_front_vertex->node().lock()->id() == left_front_node->id())
      vertex_queue.push_back(left_front_vertex);
  }

  if (right_front_vertex) {
    all_vertices_.push_back(right_front_vertex);
    if (right_front_vertex->node().lock()->id() == right_front_node->id())
      vertex_queue.push_back(right_front_vertex);
  }

  return vertex_queue;
}

void SLCLatticePlanner::constructVertexGraph(
    std::deque<boost::shared_ptr<Vertex>>& vertex_queue) {

  //std::printf("constructVertexGraph(): \n");

  auto addVertexToGraphAndQueue = [this, &vertex_queue](
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& node)->void{
    if ((!vertex) || (!node)) return;

    all_vertices_.push_back(vertex);
    if (vertex->node().lock()->id() == node->id())
      vertex_queue.push_back(vertex);
  };

  while (!vertex_queue.empty()) {
    boost::shared_ptr<Vertex> vertex = vertex_queue.front();
    vertex_queue.pop_front();

    // Try to connect to the front node.
    boost::shared_ptr<const WaypointNode> front_node =
      waypoint_lattice_->front(vertex->node().lock()->waypoint(), 50.0);
    boost::shared_ptr<Vertex> front_vertex =
      connectVertexToFrontNode(vertex, front_node);

    addVertexToGraphAndQueue(front_vertex, front_node);

    // Check if the vertex is on the same lane with the root.
    // If not, no lane change options will be allowed further.
    if (!(vertex->sameLaneWith(root_.lock()))) continue;

    // Try to connect to the left front node.
    boost::shared_ptr<const WaypointNode> left_front_node =
      waypoint_lattice_->frontLeft(vertex->node().lock()->waypoint(), 50.0);
    boost::shared_ptr<Vertex> left_front_vertex =
      connectVertexToLeftFrontNode(vertex, left_front_node);

    addVertexToGraphAndQueue(left_front_vertex, left_front_node);

    // Try to connect to the right front node.
    boost::shared_ptr<const WaypointNode> right_front_node =
      waypoint_lattice_->frontRight(vertex->node().lock()->waypoint(), 50.0);
    boost::shared_ptr<Vertex> right_front_vertex =
      connectVertexToRightFrontNode(vertex, right_front_node);

    addVertexToGraphAndQueue(right_front_vertex, right_front_node);
  }

  return;
}

boost::shared_ptr<Vertex> SLCLatticePlanner::connectVertexToFrontNode(
    const boost::shared_ptr<Vertex>& vertex,
    const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("connectVertexToFrontNode(): \n");

  // Return directly if the target node does not exist.
  if (!target_node) return nullptr;

  // Plan a path between the node at the current vertex to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::KeepLane);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created, the vertex
    // cannot be created either.
    std::printf("%s", e.what());
    return nullptr;
  }

  // Now, simulate the traffic forward with ego following the created path.
  //std::printf("Simulate the traffic.\n");
  SLCTrafficSimulator simulator(vertex->snapshot(), map_, fast_map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  try {
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);
    // There a collision is detected in the simulation, this option is ignored.
    if (!no_collision) return nullptr;
  } catch(std::exception& e) {
    std::printf("SLCLatticePlanner::connectVertexToFrontNode(): WARNING\n"
                "%s", e.what());
    return nullptr;
  }

  // A new vertex should be created.
  //std::printf("Create child vertex.\n");
  boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
      simulator.snapshot(), waypoint_lattice_, fast_map_);

  // Set the child vertex of the parent vertex.
  //std::printf("Update the child vertex of the input vertex.\n");
  vertex->updateFrontChild(*path, stage_cost, next_vertex);

  // Set the parent vertex of the child vertex.
  //std::printf("Update the parent vertex of the new vertex.\n");
  next_vertex->updateParent(
      simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);

  return next_vertex;
}

boost::shared_ptr<Vertex> SLCLatticePlanner::connectVertexToLeftFrontNode(
    const boost::shared_ptr<Vertex>& vertex,
    const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("connectVertexToLeftFrontNode(): \n");

  // Return directly if the target node does not exisit.
  if (!target_node) return nullptr;

  // Return directly if the target node is already very close to the vertex.
  // It is not reasonable to change lane with this short distance.
  if (target_node->distance()-vertex->node().lock()->distance() < 20.0)
    return nullptr;

  // Check the left front and left back vehicles.
  //
  // If there are vehicles at the left front or left back of the ego,
  // and those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> left_front =
    vertex->snapshot().trafficLattice()->leftFront(vertex->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    vertex->snapshot().trafficLattice()->leftBack(vertex->snapshot().ego().id());

  if (left_front && left_front->second <= 0.0) return nullptr;
  if (left_back  && left_back->second  <= 0.0) return nullptr;

  // Plan a path between the node at the current vertex to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::LeftLaneChange);
  } catch (const std::exception& e) {
    // If for whatever reason, the path cannot be created,
    // just ignore this option.
    std::printf("%s", e.what());
    return nullptr;
  }

  // Now, simulate the traffic forward with ego following the created path.
  //std::printf("Simulate the traffic.\n");
  SLCTrafficSimulator simulator(vertex->snapshot(), map_, fast_map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  try {
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);
    // There a collision is detected in the simulation, this option is ignored.
    if (!no_collision) return nullptr;
  } catch (std::exception& e) {
    std::printf("SLCLatticePlanner::connectVertexToLeftFrontNode(): WARNING\n"
                "%s", e.what());
    return nullptr;
  }

  // Create a new vertex.
  //std::printf("Create child vertex.\n");
  boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
      simulator.snapshot(), waypoint_lattice_, fast_map_);

  // Set the child vertex of the parent vertex.
  //std::printf("Update the child vertex of the input vertex.\n");
  vertex->updateLeftChild(*path, stage_cost, next_vertex);

  // Set the parent vertex of the child vertex.
  //std::printf("Update the parent vertex of the new vertex.\n");
  next_vertex->updateParent(
      simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);

  return next_vertex;
}

boost::shared_ptr<Vertex> SLCLatticePlanner::connectVertexToRightFrontNode(
    const boost::shared_ptr<Vertex>& vertex,
    const boost::shared_ptr<const WaypointNode>& target_node) {

  //std::printf("connectVertexToRightFrontNode(): \n");

  // Return directly if the target node does not exisit.
  if (!target_node) return nullptr;

  // Return directly if the target node is already very close to the vertex.
  // It is not reasonable to change lane with this short distance.
  if (target_node->distance()-vertex->node().lock()->distance() < 20.0)
    return nullptr;

  // Check the right front and right back vehicles.
  //
  // If there are vehicles at the right front or right back of the ego,
  // and those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> right_front =
    vertex->snapshot().trafficLattice()->rightFront(vertex->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    vertex->snapshot().trafficLattice()->rightBack(vertex->snapshot().ego().id());

  if (right_front && right_front->second <= 0.0) return nullptr;
  if (right_back  && right_back->second  <= 0.0) return nullptr;

  // Plan a path between the node at the current vertex to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::RightLaneChange);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created,
    // just ignore this option.
    std::printf("%s", e.what());
    return nullptr;
  }

  // Now, simulate the traffic forward with the ego following the created path.
  //std::printf("Simulate the traffic.\n");
  SLCTrafficSimulator simulator(vertex->snapshot(), map_, fast_map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  try {
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);
    // There a collision is detected in the simulation, this option is ignored.
    if (!no_collision) return nullptr;
  } catch (std::exception& e) {
    std::printf("SLCLatticePlanner::connectVertexToRightFrontNode(): WARNING\n"
                "%s", e.what());
    return nullptr;
  }

  // Create a new vertex.
  //std::printf("Create child vertex.\n");
  boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
      simulator.snapshot(), waypoint_lattice_, fast_map_);

  // Set the child vertex of the parent vertex.
  //std::printf("Update the child vertex of the input vertex.\n");
  vertex->updateRightChild(*path, stage_cost, next_vertex);

  // Set the parent vertex of the child vertex.
  //std::printf("Update the parent vertex of the new vertex.\n");
  next_vertex->updateParent(
      simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);

  return next_vertex;
}

const double SLCLatticePlanner::terminalSpeedCost(
    const boost::shared_ptr<Vertex>& vertex) const {

  if (vertex->hasChild()) {
    std::string error_msg(
        "SLCLatticePlanner::terminalSpeedCost(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + vertex->string());
  }

  static std::unordered_map<int, double> cost_map {
    {0, 3.0}, {1, 3.0}, {2, 2.0}, {3, 2.0}, {4, 2.0},
    {5, 2.0}, {6, 1.0}, {7, 1.0}, {8, 0.0}, {9, 0.0},
  };

  const double ego_speed = vertex->snapshot().ego().speed();
  const double ego_policy_speed = vertex->snapshot().ego().policySpeed();
  if (ego_speed < 0.0 || ego_policy_speed < 0.0) {
    std::string error_msg(
        "SLCLatticePlanner::terminalSpeedCost(): "
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

const double SLCLatticePlanner::terminalDistanceCost(
    const boost::shared_ptr<Vertex>& vertex) const {

  if (vertex->hasChild()) {
    std::string error_msg(
        "SLCLatticePlanner::terminalDistanceCost(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + vertex->string());
  }

  static std::unordered_map<int, double> cost_map {
    {0, 20.0}, {1, 20.0}, {2, 20.0}, {3, 20.0}, {4, 20.0},
    {5, 20.0}, {6, 20.0}, {7, 20.0}, {8, 10.0},  {9, 5.0},
  };

  // Find the current spatial planning horizon.
  boost::shared_ptr<const Vertex> root_child;
  if (root_.lock()->hasFrontChild())
    root_child = std::get<2>(*(root_.lock()->frontChild())).lock();
  else if (root_.lock()->hasLeftChild())
    root_child = std::get<2>(*(root_.lock()->leftChild())).lock();
  else if (root_.lock()->hasRightChild())
    root_child = std::get<2>(*(root_.lock()->rightChild())).lock();

  const double spatial_horizon =
    spatial_horizon_ - 50.0 +
    root_child->node()->distance() -
    root_.lock()->node().lock()->distance();

  const double distance = vertex->node().lock()->distance() -
                          root_.lock()->node().lock()->distance();

  const double distance_ratio = distance / spatial_horizon;
  //std::printf("vertex distance:%f spatial horizon:%f distance ratio: %f\n",
  //    distance, spatial_horizon, distance_ratio);

  if (distance_ratio >= 1.0) return 0.0;
  else return cost_map[static_cast<int>(distance_ratio*10.0)];
}

const double SLCLatticePlanner::costFromRootToTerminal(
    const boost::shared_ptr<Vertex>& terminal) const {

  if (terminal->hasChild()) {
    std::string error_msg(
        "SLCLatticePlanner::costFromRootToTerminal(): "
        "The input vertex is not a terminal.\n");
    throw std::runtime_error(error_msg + terminal->string());
  }

  const double path_cost = terminal->costToCome();
  const double terminal_speed_cost = terminalSpeedCost(terminal);
  const double terminal_distance_cost = terminalDistanceCost(terminal);
  //std::printf("path cost: %f speed cost: %f distance cost:%f\n",
  //    path_cost, terminal_speed_cost, terminal_distance_cost);

  // TODO: Weight the cost properly.
  return path_cost + terminal_speed_cost + terminal_distance_cost;
}

void SLCLatticePlanner::selectOptimalPath(
    std::list<ContinuousPath>& path_sequence,
    std::list<boost::weak_ptr<Vertex>>& vertex_sequence) const {

  //std::printf("selectOptimalPath():\n");

  //std::printf("Find optimal terminal vertex.\n");
  // Find the optimal terminal vertex.
  boost::shared_ptr<Vertex> optimal_vertex = nullptr;
  double optimal_cost = 1.0e10;

  for (const auto& vertex : all_vertices_) {
    if (vertex->hasChild()) continue;
    const double vertex_cost = costFromRootToTerminal(vertex);

    // Update the terminal vertex if the candidate has smaller cost.
    if (vertex_cost < optimal_cost) {
      optimal_vertex = vertex;
      optimal_cost = vertex_cost;
    }
  }

  // The optimal vertex should be set no matter what.
  if (!optimal_vertex) {
    throw std::runtime_error(
        "SLCLatticePlanner::selectOptimalPath(): "
        "no terminal vertex in the graph.\n");
  }

  // There should always be a parent vertex for a terminal vertex.
  if (!optimal_vertex->hasParent()) {
    throw std::runtime_error(
        "SLCLatticePlanner::selectOptimalPath(): "
        "the graph only has root vertex.\n");
  }

  // Lambda functions to get the child vertex IDs given a parent vertex.
  auto frontChildId = [this](const boost::shared_ptr<Vertex>& vertex)->boost::optional<size_t>{
    if (!(vertex->frontChild())) return boost::none;
    else return std::get<2>(*(vertex->frontChild())).lock()->node().lock()->id();
  };
  auto leftChildId = [this](const boost::shared_ptr<Vertex>& vertex)->boost::optional<size_t>{
    if (!(vertex->leftChild())) return boost::none;
    else return std::get<2>(*(vertex->leftChild())).lock()->node().lock()->id();
  };
  auto rightChildId = [this](const boost::shared_ptr<Vertex>& vertex)->boost::optional<size_t>{
    if (!(vertex->rightChild())) return boost::none;
    else return std::get<2>(*(vertex->rightChild())).lock()->node().lock()->id();
  };

  // Trace back from the terminal vertex to find all the paths.
  path_sequence.clear();
  vertex_sequence.clear();

  boost::shared_ptr<Vertex> vertex = optimal_vertex;
  vertex_sequence.push_front(boost::weak_ptr<Vertex>(vertex));

  //std::printf("Trace back parent vertex.\n");
  while (vertex->hasParent()) {

    boost::shared_ptr<Vertex> parent_vertex =
      std::get<2>((*(vertex->parent()))).lock();
    if (!parent_vertex) {
      std::string error_msg(
          "SLCLatticePlanner::selectOptimalPath(): "
          "cannot find parent when tracing back optimal path from the vertex.\n");
      throw std::runtime_error(error_msg + vertex->string());
    }

    // Insert the parent vertex to the queue.
    vertex_sequence.push_front(boost::weak_ptr<Vertex>(parent_vertex));

    // Insert the path between the parent and this vertex to the queue.
    if (frontChildId(parent_vertex) &&
        frontChildId(parent_vertex) == vertex->node().lock()->id()) {
      path_sequence.push_front(std::get<0>(*(parent_vertex->frontChild())));
      vertex = parent_vertex;
      continue;
    }

    if (leftChildId(parent_vertex) &&
        leftChildId(parent_vertex) == vertex->node().lock()->id()) {
      path_sequence.push_front(std::get<0>(*(parent_vertex->leftChild())));
      vertex = parent_vertex;
      continue;
    }

    if (rightChildId(parent_vertex) &&
        rightChildId(parent_vertex) == vertex->node().lock()->id()) {
      path_sequence.push_front(std::get<0>(*(parent_vertex->rightChild())));
      vertex = parent_vertex;
      continue;
    }

  }

  return;
}

DiscretePath SLCLatticePlanner::mergePaths(
    const std::list<ContinuousPath>& paths) const {

  DiscretePath path(paths.front());
  for (std::list<ContinuousPath>::const_iterator iter = ++(paths.begin());
       iter != paths.end(); ++iter) path.append(*iter);
  return path;
}

} // End namespace slc_lattice_planner.
} // End namespace planner.
