#pragma once
#include <vector>
#include <functional>

// The type of the function f(t, y) that defines the ODE
// It takes a double (time) and a vector (state) and returns a vector
using ODEFunc = std::function<
    std::vector<double>(double, const std::vector<double>&)
>;

class RK4Solver
{
public:
    // Constructor: takes the ODE function and step size h
    RK4Solver(ODEFunc f, double h);

    // Take a single RK4 step from time t with state y
    // Returns the new state at time t + h
    std::vector<double> step(
        double t,
        const std::vector<double>& y
    );

    // Solve from t0 to t1 with initial condition y0
    // Returns the solution trajectory as a 2D vector
    // trajectory[i] = state at time t0 + i*h
    std::vector<std::vector<double>> solve(
        double t0,
        double t1,
        const std::vector<double>& y0
    );

private:
    ODEFunc m_f;  // the ODE function f(t, y)
    double  m_h;  // the step size
};
