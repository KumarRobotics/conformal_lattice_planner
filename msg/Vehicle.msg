# ID of the vehicle
uint64 id
# Bounding box of the vehicle.
conformal_lattice_planner/BoundingBox bounding_box
# Left-hand transform of the vehicle, compatible with Carla simulator.
geometry_msgs/Pose transform
# Speed
float64 speed
# Acceleration
float64 acceleration
# Curvature of the path where the vehicle is at.
float64 curvature
# Policy speed
float64 policy_speed

