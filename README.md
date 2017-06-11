Denise R. James

Udacity Model Predictive Control (MPC) - Project 5

1.  The Model Predictive Control Architecture.
This project defines a non linear model predictive control system using the Unity simulator that provides the vehicle’s global 2D location, x and y, vehicle global orientation angle between x and y, called psi, a yellow projected reference line called waypoints in global coordinates and the velocity of the vehicle.  The waypoints are converted to car coordinate space.  The converted waypoints are fit to a third order polynomial.  The cross track error, cte, is defined as the the coefficients of the previous third order polynomial evaluated. The orientation error, epsi, is defined as the change in error, derivate, of the vehicle’s movement.  The state vector is defined by using the car coordinates that are x=0, y=0 and psi=0.  Velocity, v, is used from being provided by the simulator.  cte and epsi as calculated above using the Eigen header files, state << 0., 0., 0., v, cte, epsi

The update next state is defined by using the ipopt library, Solve function.  The inputs are the state and the coefficients.  The car coordinates will always be x=0, y=0 and psi =0 as it is the reference point.  The reference waypoint is displayed as a yellow line in the simulator.

The following updated values are sent to the simulator:  steering angle, throttle, x, y, and the waypoints ptsx, ptsy.  The updated waypoints are displayed as a green line.  The goal is to have the yellow and green lines match. 

2. Timestep Length and Elapsed Duration (N & dt)

I initially used the lecture N=25 and dt =0.05 values.  The car ran off the road at N = 25.  After trying lower N values, I settled on N=10.  I choose to keep dt=0.05 since 2*dt = 100 microseconds, the latency I must implement.  

3. Polynomial Fitting and MPC Preprocessing

Polynomial fitting is done after the global waypoints are converted to car space

A.  Convert to car space
c_x.resize(m_x.size());
        c_y.resize(m_y.size());

        const auto cosa = cos(psi);
        const auto sina = sin(psi);

        for (int i = 0; i < c_x.size(); i++) {
            const auto x = m_x[i] - tx;
            const auto y = m_y[i] - ty;
            c_x[i] = x * cosa + y * sina;
            c_y[i] = -x * sina + y * cosa;
        }
B.  Polynomial fitting

// Third order polyfit function
            coeffs = polyfit(x, y, 3);
// The cross track error is calculated by cte = polyeval(coeffs)
            double cte = polyeval(coeffs,0);
Details of the code is found in source code.


4.  Model Predictive Control with Latency

Latency is a challenge in this project.  I settled on implementing the 100 microsecond delay as required.  Since dt = 0.05 seconds, then 2*dt = 100 microsecond.  This is two dt time intervals.  I choose to implement the latency in the Solve function.  The steering and acceleration does not change during the 100 microseconds latency.  The previous value of steering and acceleration is assigned during this latency period.
         

A video of this project is found https://www.youtube.com/watch?v=Q3rMqWWroQ0&feature=youtu.be .
