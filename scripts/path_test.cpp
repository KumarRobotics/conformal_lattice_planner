
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath>

#include <boost/format.hpp>
#include "path_optimization.h"

using namespace planner;
using namespace std;

int main(int argc, char** argv) {

  NonHolonomicPath::State lc_start(14.254, 125.566, -96.9345/(2*M_PI), 0.0461623);
  NonHolonomicPath::State lc_end(12.2967, 75.585, -90.2891/(2*M_PI), 0.0);

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

  return 0;
}

