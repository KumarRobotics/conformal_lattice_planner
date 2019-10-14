//
// Created by brent on 9/11/19.
//

#ifndef KN_PATH_GEN_KN_PATH_GEN_H
#define KN_PATH_GEN_KN_PATH_GEN_H

#include <vector>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>
//#include <json/json.h>

//using namespace Eigen;

namespace planner {

class NonHolonomicPath {
  /**
   * The NonHolonomicPath represents parametrized curves determined by five parameters, namely
   * a, b, c, d, and sf, corresponding to a spatial path determined by a parametrized curvature
   * function K(s) = a + b*s + c * s ^2 + d * s^3. The NonHolonomicPath can compute waypoints at
   * arbitrary points along the curve.
   */

 public:

  struct State {
    /**
     * The State class represents a tuple of (x, y, theta, kappa) which is the state
     * space for the Kelly Nagy Path generation method.
     */
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double kappa{0.0};

    /// Default constructor.
    State() = default;

    /**
     * Create a State struct.
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @param theta The heading angle in the inertial frame.
     * @param kappa The curvature in the inertial frame.
     */
    State(double x, double y, double theta, double kappa) : x(x), y(y), theta(theta), kappa(kappa) {}

    /**
     * Convert the state to a vector representation.
     * @return The vector representation of the state.
     */
    Eigen::Vector4d toVector() const {
      return {x, y, theta, kappa};
    }
  };

 public:
  // Curvature Polynomial Coefficients
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double d{0.0};

  double sf{0.0}; // The total length of the path in meters.

  /// Default constructor.
  NonHolonomicPath() = default;

  /**
   * Construct a NonHolonomicPath from a given set of coefficients.
   * @param a The constant coefficient of the curvature polynomial.
   * @param b The linear coefficient of the curvature polynomial.
   * @param c The quadratic coefficient of the curvature polynomial.
   * @param d The cubic coefficient of the curvature polynomial.
   * @param sf The path length of the entire path.
   */
  NonHolonomicPath(double a, double b, double c, double d, double sf) : a(a), b(b), c(c), d(d), sf(sf) {}

  /**
   * Evaluate a waypoint along the path corresponding to a given arc-length parameter s.
   * @param x0 The initial state to evaluate the curve with respect to.
   * @param s The arc length along the path.
   * @return The resulting state.
   */
  State evaluate(State x0, double s) const {

    // Verify the Path curvature matches the initial condition.
    assert(std::abs(a - x0.kappa) < 1e-5);

    using std::pow;
    // Curvature and Theta have closed form expressions.
    double kappa = a + b * s + c * pow(s, 2) + d * pow(s, 3); // K(s) = a + b*s + c*s^2 + d*s^3
    double theta = a * s + b * pow(s, 2) / 2 +
        c * pow(s, 3) / 3 + d * pow(s, 4) / 4; // Th(s) = a*s + b*s^2/2 + c*s^3/3 + d * s^4/4

    // Now Approximate X, and Y via Simpson's rule.
    double x = 0.0;
    double y = 0.0;
    double N = 101;

    // Eigen Arrays here for coefficient wise operations.
    Eigen::ArrayXd s_arr = Eigen::ArrayXd::LinSpaced(N, 0, s);
    Eigen::ArrayXd theta_arr = a * s_arr + b * s_arr.pow(2) / 2 + c * s_arr.pow(3) / 3 + d * s_arr.pow(4) / 4;
    Eigen::ArrayXd cos_arr = theta_arr.cos();
    Eigen::ArrayXd sin_arr = theta_arr.sin();

    x = simpsonsRule(cos_arr, s_arr);
    y = simpsonsRule(sin_arr, s_arr);

    // Transform to Global Frame
    Eigen::Matrix3d R; // Homogenous Coordinates Transformation
    R << std::cos(x0.theta), -std::sin(x0.theta), x0.x,
         std::sin(x0.theta),  std::cos(x0.theta), x0.y,
                          0,                   0,    1;
    Eigen::Vector3d x1 {x, y, 1};
    Eigen::Vector3d x2 = R * x1;

    return {x2[0], x2[1], unrollAngle(theta + x0.theta), kappa}; // Construct a State object to return the resulting waypoint.
  }

  /**
   * Computes a simple method for Simpson's Rule for numerical integration.
   * @param f The function evaluated at the discrete points.
   * @param dx The points where the function is evaluated at. Note that this simply extracts the start, end, and width
   * of the interval.
   * @return The approximate value of the definite integral evaluated according to Simpson's rule.
   */
  double simpsonsRule(const Eigen::ArrayXd &f, const Eigen::ArrayXd &dx) const {
    int N = dx.size();
    double a = dx(0);
    double b = dx(N - 1);
    double h = (b - a) / N;

    // Compute the weights W.
    Eigen::ArrayXd W = Eigen::ArrayXd::Ones(N);
    //std::cout <<"Before modifying " << W << std::endl;
    Eigen::Map<Eigen::ArrayXd, 0, Eigen::InnerStride<2>> even(W.data() + 2, W.size() / 2 - 1);
    Eigen::Map<Eigen::ArrayXd, 0, Eigen::InnerStride<2>> odd(W.data() + 1, W.size() / 2);
    even = even * 2;
    odd = odd * 4;
    //std::cout <<"After modifying" << W << std::endl;

    double result = h / 3 * W.matrix().transpose() * f.matrix(); // Compute the Simpson's Rule result.

    return result;
  }

  /**
  * Optimize the path with respect to the initial and final state constraints, using
   * the exact solution of linear constraints method for improved speed.
  * @param x0 The initial state constraint.
  * @param xf The final state constraint.
  * @param iterations The maximum number of iterations.
  * @return True if the optimization has converged.
  */
  bool optimizePath(const State &x0, const State &xf, unsigned iterations = 100) {

    bool result = true;

    // Transform to Local Frame
    Eigen::Matrix3d R; // Homogenous Coordinates Transformation
    R << std::cos(x0.theta), -std::sin(x0.theta), x0.x,
         std::sin(x0.theta),  std::cos(x0.theta), x0.y,
                          0,                   0,    1;
    State x0_L {0.0, 0.0, 0.0, x0.kappa};
    Eigen::Vector3d x1 {xf.x, xf.y, 1};
    Eigen::Vector3d x2 = R.inverse() * x1;
    State xf_L {x2[0], x2[1], unrollAngle(xf.theta - x0.theta), xf.kappa};

    NonHolonomicPath initial_guess = initialGuess(x0_L, xf_L);
    a = initial_guess.a;
    b = initial_guess.b;
    c = initial_guess.c;
    d = initial_guess.d;
    sf = initial_guess.sf;

    using std::pow;
    size_t counter = 0;
    for (; counter < iterations; ++counter) {
      Eigen::Vector4d old_path{b, c, d, sf};

      Eigen::Matrix4d J = boundaryConstraintJacobian(x0_L, xf_L);
      Eigen::Vector4d g = boundaryConstraint(x0_L, xf_L);

      Eigen::Vector4d dq = J.colPivHouseholderQr().solve(-g);
      b += dq[0];
      sf += dq[3];
      //c += dq[1];
      //d += dq[2];

      // Solve Linear Equations A_x = b_ to compute exact solutions for c, and d.
      Eigen::Matrix2d A_;
      A_ << pow(sf, 2), pow(sf, 3), pow(sf, 3) / 3, pow(sf, 4) / 4;
      Eigen::Vector2d b_;
      b_ << xf_L.kappa - x0_L.kappa      - b * sf,
            xf_L.theta - x0_L.kappa * sf - b * pow(sf, 2) / 2;
      Eigen::Vector2d cd = A_.colPivHouseholderQr().solve(b_);
      c = cd[0];
      d = cd[1];

      // Check for Convergence
      Eigen::Vector4d new_path{b, c, d, sf};
      if ((old_path - new_path).norm() < 1e-3) {
        break; // Tolerance
      }
    }

    // If max iteration reached, the optimization has probably diverged.
    if (counter >= iterations) return false;

    // If the computed final state is far from the required final state,
    // the optimization has diverged. Here we only use the difference ratio
    // in x and y directions.
    auto xf_L_test = evaluate(x0_L, sf);
    // FIXME: Use both ratio and absolute threshold to check convergence.
    if (std::fabs(xf_L_test.x-xf_L.x) / std::fabs(xf_L.x) > 0.2 ||
        std::fabs(xf_L_test.y-xf_L.y) / std::fabs(xf_L.y) > 0.2) return false;

    return true;
  }
 private:

  /**
   * Compute the Jacobian matrix of the boundary constraint set with respect to the initial and final constraints.
   * @param x0 The initial constraint.
   * @param xf The final constraint.
   * @return The resulting Jacobian matrix.
   */
  Eigen::Matrix4d boundaryConstraintJacobian(const State &x0, const State &xf) const {

    double N = 101;
    Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();

    using std::pow; // For convenience.

    // Curvature and Theta have closed form expressions.
    double kappa_f = a + b * sf + c * pow(sf, 2) + d * pow(sf, 3); // K(sf) = a + b*sf + c*sf^2 + d*sf^3
    double theta_f = a * sf + b * pow(sf, 2) / 2 +
        c * pow(sf, 3) / 3 + d * pow(sf, 4) / 4; // Th(sf) = a*sf + b*sf^2/2 + c*sf^3/3 + d * sf^4/4

    // Eigen Arrays here for coefficient wise operations.
    Eigen::ArrayXd s_arr = Eigen::ArrayXd::LinSpaced(N, 0, sf);
    Eigen::ArrayXd theta_arr = a * s_arr + b * s_arr.pow(2) / 2 + c * s_arr.pow(3) / 3 + d * s_arr.pow(4) / 4;

    Eigen::ArrayXd cos2_arr = s_arr.pow(2) * theta_arr.cos();
    Eigen::ArrayXd cos3_arr = s_arr * cos2_arr;
    Eigen::ArrayXd cos4_arr = s_arr * cos3_arr;

    Eigen::ArrayXd sin2_arr = s_arr.pow(2) * theta_arr.sin();
    Eigen::ArrayXd sin3_arr = s_arr * sin2_arr;
    Eigen::ArrayXd sin4_arr = s_arr * sin3_arr;

    // dx/dq
    double S2 = simpsonsRule(sin2_arr, s_arr);
    double S3 = simpsonsRule(sin3_arr, s_arr);
    double S4 = simpsonsRule(sin4_arr, s_arr);
    jacobian.row(0) << -S2 / 2, -S3 / 3, -S4 / 4, std::cos(theta_f);

    // dy/dq
    double C2 = simpsonsRule(cos2_arr, s_arr);
    double C3 = simpsonsRule(cos3_arr, s_arr);
    double C4 = simpsonsRule(cos4_arr, s_arr);
    jacobian.row(1) << C2 / 2, C3 / 3, C4 / 4, std::sin(theta_f);

    // dth/dq
    jacobian.row(2) << pow(sf, 2) / 2, pow(sf, 3) / 3, pow(sf, 4) / 4, kappa_f;

    // dk/dq
    jacobian.row(3) << sf, pow(sf, 2), pow(sf, 3), b + 2 * c * sf + 3 * d * pow(sf, 2);

    return jacobian;
  }

  /**
   * Evaluate the boundary constraint values of the current path given the initial and final constraints.
   * @param x0 The initial constraint.
   * @param xf The final constraint.
   * @return A vector representing the value of all constraints.
   */
  Eigen::Vector4d boundaryConstraint(const State &x0, const State &xf) const {
    State xf_g = evaluate(x0, sf); // The endpoint of the current path.

    Eigen::Vector4d g = xf_g.toVector() - xf.toVector();
    g[2] = shortestAngle(xf_g.theta, xf.theta); // Deal with Angle wrap-around issues.
    return g;
  }

  /**
   * Generate an initial guess of the NonHolonomicPath coefficients given an initial and terminal state.
   * @param x0 The initial state.
   * @param xf The terminal state.
   * @return The initial guess of the path.
   */
  static NonHolonomicPath initialGuess(const State &x0, const State &xf) {
    double theta = shortestAngle(x0.theta, xf.theta);
    double sf = std::pow(theta, 2) / 5.0 + 1.0;
    double eta = std::hypot(x0.x - xf.x, x0.y - xf.y); // Scaling for the path length.
    // Create the initial Path.
    NonHolonomicPath path = NonHolonomicPath(x0.kappa, 0.0, 0.0, 0.0, sf * eta);
    return path;
  }
  /**
   * Computes the minimum angle between the two specified angles.
   * @param angle1 The first angle.
   * @param angle2 The second angle.
   * @return The distance between the two angles.
   */
  static double shortestAngle(double angle1, double angle2) {
    angle1 = unrollAngle(angle1);
    angle2 = unrollAngle(angle2);

    double diff = angle1 - angle2;
    if (std::fabs(diff+2*M_PI) < std::fabs(diff)) diff += 2*M_PI;
    if (std::fabs(diff-2*M_PI) < std::fabs(diff)) diff -= 2*M_PI;

    return diff;
  }

  /**
   * Unroll an angle into the interval [-pi, pi].
   * @param angle The angle to be unrolled.
   * @return The value when restricted to [-pi, pi].
   */
  static double unrollAngle(double angle) {
    //return std::fmod(angle + M_PI, (2 * M_PI)) - M_PI;
    angle = std::remainder(angle, 2.0*M_PI);
    if (angle < -M_PI) angle += 2.0*M_PI;
    if (angle >  M_PI) angle -= 2.0*M_PI;
    return angle;
  }
};

} // End namespace planner.

/****************
    Overloaded Output Streams for KN Path Classes.
 ****************/
inline std::ostream &operator<<(std::ostream &os, const planner::NonHolonomicPath::State &s) {
  os << "State (x=" << s.x << ", y=" << s.y << ", theta=" << s.theta << ", kappa=" << s.kappa;
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const planner::NonHolonomicPath &p) {
  os << "NonHolonomicPath (a=" << p.a
     << ", b=" << p.b
     << ", c=" << p.c
     << ", d=" << p.d
     << ", sf=" << p.sf
     << ")";
  return os;
}

#endif //KN_PATH_GEN_KN_PATH_GEN_H
