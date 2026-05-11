#pragma once

#include <vector>
#include <functional>
#include <utility>

// ─────────────────────────────────────────────
// ODEFunc: the type of f(t, y)
// Takes time t and state vector y
// Returns the derivative dy/dt
// ─────────────────────────────────────────────
using ODEFunc = std::function<
    std::vector<double>(double, const std::vector<double>&)
>;

// ─────────────────────────────────────────────
// RK4Solver: Fixed step size solver
// Uses the classical 4th order Runge-Kutta method
// Good for problems where uniform step size is OK
// ─────────────────────────────────────────────
class RK4Solver
{
public:
    // Constructor
    // f: the ODE function f(t, y)
    // h: fixed step size
    RK4Solver(ODEFunc f, double h);

    // Take a single RK4 step from time t with state y
    // Returns the new state at time t + h
    std::vector<double> step(
        double t,
        const std::vector<double>& y
    );

    // Solve from t0 to t1 with initial condition y0
    // Returns full trajectory: trajectory[i] = state at step i
    std::vector<std::vector<double>> solve(
        double t0,
        double t1,
        const std::vector<double>& y0
    );

private:
    ODEFunc m_f;  // the ODE function
    double  m_h;  // fixed step size
};

// ─────────────────────────────────────────────
// RK45Solver: Adaptive step size solver
// Uses Dormand-Prince method (embedded RK4/RK5)
// Automatically adjusts step size to meet
// user-specified accuracy tolerances
// Better than RK4 for problems needing high accuracy
// ─────────────────────────────────────────────
class RK45Solver
{
public:
    // Constructor
    // f:      the ODE function f(t, y)
    // rtol:   relative tolerance (default 1e-3)
    // atol:   absolute tolerance (default 1e-6)
    // h_init: initial step size (default 0.1)
    // h_max:  maximum allowed step size (default 1.0)
    // h_min:  minimum allowed step size (default 1e-10)
    RK45Solver(ODEFunc f,
               double rtol   = 1e-3,
               double atol   = 1e-6,
               double h_init = 0.1,
               double h_max  = 1.0,
               double h_min  = 1e-10);

    // Solve from t0 to t1 with initial condition y0
    // Returns full trajectory with adaptive time steps
    std::vector<std::vector<double>> solve(
        double t0,
        double t1,
        const std::vector<double>& y0
    );

    // Get the time points where solution was evaluated
    std::vector<double> get_times() const { return m_times; }

    // Get number of accepted steps
    int get_n_steps() const { return m_n_steps; }

    // Get number of rejected steps (too much error)
    int get_n_rejected() const { return m_n_rejected; }

private:
    ODEFunc m_f;
    double  m_rtol;
    double  m_atol;
    double  m_h_init;
    double  m_h_max;
    double  m_h_min;
    int     m_n_steps;
    int     m_n_rejected;
    std::vector<double> m_times;

    // Single RK45 step using Dormand-Prince coefficients
    // Returns: {new_state, error_norm}
    // error_norm <= 1.0 means step is accurate enough
    std::pair<std::vector<double>, double> step(
        double t,
        const std::vector<double>& y,
        double h
    );
};