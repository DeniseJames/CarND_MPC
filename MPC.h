#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  ~MPC();
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  static constexpr double max_delta = 0.436332;

  static const int latency_unit;

  double steering_value() const { return -steering_delta_ / max_delta; }
  double throttle_value() const { return a_; }

  std::vector<double> x_predicted_value_;
  std::vector<double> y_predicted_value;



private:
  double steering_delta_;
  double a_;
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.



};

#endif /* MPC_H */
