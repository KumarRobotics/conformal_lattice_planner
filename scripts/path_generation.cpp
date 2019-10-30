
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

#include <boost/format.hpp>
#include "path_optimization.h"

using namespace planner;
using namespace std;

int main(int argc, char** argv) {

  NonHolonomicPath::State lc_start(0.0, 0.0, 0.0, 0.0);
  NonHolonomicPath::State lc_end(50.0, 3.7, 0.0, 0.0);

  NonHolonomicPath path;
  const bool success = path.optimizePath(lc_start, lc_end);



  if (!success) {
    boost::format state_format("%1% %2% %3% %4%\n");
    std::string error_msg("Path optimization failed.\n");
    error_msg += "lc start state (x y theta kappa): " + (
        state_format % lc_start.x
                     % lc_start.y
                     % lc_start.theta
                     % lc_start.kappa).str();
    error_msg += "lc end state (x y theta kappa): " + (
        state_format % lc_end.x
                     % lc_end.y
                     % lc_end.theta
                     % lc_end.kappa).str();
    throw std::runtime_error(error_msg);
    return -1;
  }

  boost::format waypoint_format("%1% %2% %3% %4% %5%\n");
  std::string path_msg;
  for (double s = 0.0; s <= path.sf; s+=0.05) {
    NonHolonomicPath::State state = path.evaluate(lc_start, s);
    path_msg += (waypoint_format % s
                                 % state.x
                                 % state.y
                                 % state.theta
                                 % state.kappa).str();
  }

  ofstream fout;
  fout.open("left_lane_change_path");
  fout << path_msg;
  fout.close();

  return 0;
}
