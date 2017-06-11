#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.05;

constexpr double coeff_derivative_delta = 500; // increase smoothness driving (smoothness steering)
constexpr double coeff_derivative_a = 1.;      // increase smoothness of acceleration
constexpr double coeff_penalize_delta = 200;   // minimizes the use of steering.
constexpr double coeff_penalize_a = 1.;	       // minimizes the use of acceleration.

//**************************************************************************
// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 75 mph.
double constexpr ref_cte_ = 0;
double constexpr ref_epsi_ = 0;
double constexpr ref_v_ = 75;


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// latency is 100micros as dt = 0.05 secs *2 = 100 microseconds
// therefore latency_unit is 2 steps in vars
const int MPC::latency_unit = 2;

// assign the var array numbers.  If N=10 each will have ten
// spaces except the actuators that will have N-1 = 10
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

//**************************************************************************
class FG_eval
{
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars)
    {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (int i = 0; i < N; i++) {
            fg[0] += CppAD::pow(vars[cte_start +  i] - ref_cte_, 2);
            fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi_, 2);
            fg[0] += CppAD::pow(vars[v_start +    i] - ref_v_, 2);
            //cout<<" ref_v is  "<< ref_v << endl;
            // cout<< "pow(vars[cte_start +  i] - ref_cte, 2);  "<< pow(vars[cte_start +  i] - ref_cte, 2)<<endl;
            //cout<< "pow(vars[epsi_start +  i] - ref_cte, 2);  "<< pow(vars[epsi_start +  i] - ref_epsi, 2)<<endl;

            // cout<< "pow(vars v_start +  i] - ref_cte, 2);  "<< pow(vars[v_start +  i] - ref_v, 2)<<endl;


        }
        //cout << "First fg[0] cost function:  "<< fg[0]<<endl;

        // Minimize the use of actuators.
        for (int i = 0; i < N - 1; i++) {
            fg[0] += coeff_penalize_delta * CppAD::pow(vars[delta_start + i], 2);
            fg[0] += coeff_penalize_a * CppAD::pow(vars[a_start + i], 2);
        }
        //  cout<<"coeff_penalize_delta  "<< coeff_penalize_delta<< endl;
        //cout<<"coeff_derivative_delta  "<< coeff_derivative_delta<< endl;
        //cout<<"coeff_penalize_a  "<< coeff_penalize_a<< endl;
        //cout<<"coeff_penalize_a  "<< coeff_derivative_a<< endl;
        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
            fg[0] += coeff_derivative_delta * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += coeff_derivative_a * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        }
        //cout<< " FG[0] " << fg[0] << endl;
        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int i = 0; i < N - 1; i++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];
            AD<double> psi1 = vars[psi_start + i + 1];
            AD<double> v1 = vars[v_start + i + 1];
            AD<double> cte1 = vars[cte_start + i + 1];
            AD<double> epsi1 = vars[epsi_start + i + 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> psi0 = vars[psi_start + i];
            AD<double> v0 = vars[v_start + i];
            AD<double> cte0 = vars[cte_start + i];
            AD<double> epsi0 = vars[epsi_start + i];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + i];
            AD<double> a0 = vars[a_start + i];

            // use 3rd order polynomial
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
            AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0 + 3 * coeffs[3]*x0*x0);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[2 + x_start    + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start    + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start  + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_start    + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start  + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};


//
// MPC class definition implementation.
//
MPC::MPC() {
    steering_delta_ = 0.0;
    a_ = 0.0;
    x_predicted_value_.resize(N-1);
    y_predicted_value_.resize(N-1);
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    //size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    Eigen::VectorXd& x0 = state;

    double x = x0[0];
    double y = x0[1];
    double psi = x0[2];
    double v = x0[3];
    double cte = x0[4];
    double epsi = x0[5];

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9

    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 6 + (N - 1) * 2;
    // Number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -max_delta;// -0.436332;
        vars_upperbound[i] = max_delta;// 0.436332;
    }

    // Steering angle is not changed during latency
    for (int i = delta_start; i < delta_start + latency_unit; i++) {
        vars_lowerbound[i] = steering_delta_;
        vars_upperbound[i] = steering_delta_;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Acceleration/deceleration is not changed during latency
    for (int i = a_start; i < a_start + latency_unit; i++) {
        vars_lowerbound[i] = a_;
        vars_upperbound[i] = a_;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.05\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values: { x, y, psi, v, cte, epsi, delta, a }

    // The first actuator vector after latency is used.
    steering_delta_ = solution.x[delta_start + latency_unit];
    a_ = solution.x[a_start + latency_unit];
    cout<<"delta_start   "<<delta_start<< endl;
    cout<<"steering_delta is "<< steering_delta_ << "  a_  "<< a_<< endl;

/**/
    for (int i = 1; i < N; i++) {
        x_predicted_value_[i-1] = solution.x[x_start + i];
        y_predicted_value_[i-1] = solution.x[y_start + i];



    }
    return { solution.x[x_start   + 1], solution.x[y_start    + 1],
             solution.x[psi_start + 1], solution.x[v_start    + 1],
             solution.x[cte_start + 1], solution.x[epsi_start + 1],
             solution.x[delta_start + 1], solution.x[a_start + 1 ] };
}
