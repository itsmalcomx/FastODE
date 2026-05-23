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
// Trajectory: flat contiguous 2D array
//
// BEFORE (jagged array — bad for cache):
//   vector<vector<double>> — each row is a
//   separate heap allocation at random address
//
// AFTER (flat array — cache friendly):
//   Single contiguous block of memory
//   Access: data[step * n_dims + dim]
//   All steps stored next to each other in RAM
// ─────────────────────────────────────────────
class Trajectory
{
public:
    Trajectory() : m_n_dims(0), m_n_steps(0) {}

    // Initialize with known dimensions
    Trajectory(size_t n_dims, size_t capacity)
        : m_n_dims(n_dims)
        , m_n_steps(0)
    {
        // Allocate one flat block for all steps
        // capacity * n_dims doubles in one contiguous block
        m_data.reserve(capacity * n_dims);
    }

    // Add a new step to the trajectory
    // Copies state vector into the flat array
    void push_back(const std::vector<double>& state)
    {
        for (double v : state)
            m_data.push_back(v);
        ++m_n_steps;
    }

    // Access element at [step][dim]
    // step * n_dims + dim gives the flat index
    double operator()(size_t step, size_t dim) const
    {
        return m_data[step * m_n_dims + dim];
    }

    // Get entire step as a vector (for compatibility)
    std::vector<double> get_step(size_t step) const
    {
        std::vector<double> result(m_n_dims);
        for (size_t d = 0; d < m_n_dims; ++d)
            result[d] = m_data[step * m_n_dims + d];
        return result;
    }

    // Raw pointer to data — for NumPy buffer protocol
    const double* data() const { return m_data.data(); }
    double*       data()       { return m_data.data(); }

    size_t n_dims()  const { return m_n_dims; }
    size_t n_steps() const { return m_n_steps; }
    size_t size()    const { return m_data.size(); }

    // Convert to vector<vector<double>> for compatibility
    std::vector<std::vector<double>> to_nested() const
    {
        std::vector<std::vector<double>> result(m_n_steps);
        for (size_t s = 0; s < m_n_steps; ++s)
            result[s] = get_step(s);
        return result;
    }

    // Get the last step — replaces vector::back()
std::vector<double> back() const
{
    return get_step(m_n_steps - 1);
}

private:
    std::vector<double> m_data;  // flat contiguous storage
    size_t m_n_dims;             // number of ODE dimensions
    size_t m_n_steps;            // number of time steps stored
};

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
    // std::vector<std::vector<double>> solve(
    //     double t0,
    //     double t1,
    //     const std::vector<double>& y0
    // );
    Trajectory solve(
    double t0, double t1,
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
    // std::vector<std::vector<double>> solve(
    //     double t0,
    //     double t1,
    //     const std::vector<double>& y0
    // );
    Trajectory solve(
    double t0, double t1,
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

    // ── Pre-allocated workspace vectors ──────────────────
    // Allocated ONCE when solve() is first called
    // Reused every step — avoids repeated heap allocation
    // 'mutable' allows modification even in const methods
    mutable std::vector<double> m_k1, m_k2, m_k3;
    mutable std::vector<double> m_k4, m_k5, m_k6, m_k7;
    mutable std::vector<double> m_y_new;
    mutable std::vector<double> m_err;
    mutable std::vector<double> m_tmp;

    // Single RK45 step using Dormand-Prince coefficients
    // Returns: {new_state, error_norm}
    // error_norm <= 1.0 means step is accurate enough
    std::pair<std::vector<double>, double> step(
        double t,
        const std::vector<double>& y,
        double h
    );
};