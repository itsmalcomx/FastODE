#include "rk4.hpp"

// Helper: add two vectors element-wise
// e.g. {1,2} + {3,4} = {4,6}
static std::vector<double> vec_add(
    const std::vector<double>& a,
    const std::vector<double>& b)
{
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + b[i];
    return result;
}

// Helper: multiply a vector by a scalar
// e.g. 0.5 * {2,4} = {1,2}
static std::vector<double> vec_scale(
    double scalar,
    const std::vector<double>& v)
{
    std::vector<double> result(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        result[i] = scalar * v[i];
    return result;
}

// Constructor: store the function and step size
RK4Solver::RK4Solver(ODEFunc f, double h)
    : m_f(f), m_h(h)
{}

// Single RK4 step
// The RK4 formula uses 4 "stages" (k1, k2, k3, k4)
// Each stage is an estimate of the slope at different points
std::vector<double> RK4Solver::step(
    double t,
    const std::vector<double>& y)
{
    // k1: slope at the beginning of the step
    auto k1 = vec_scale(m_h, m_f(t, y));

    // k2: slope at the midpoint using k1
    auto k2 = vec_scale(m_h, m_f(
        t + m_h/2,
        vec_add(y, vec_scale(0.5, k1))
    ));

    // k3: slope at the midpoint using k2
    auto k3 = vec_scale(m_h, m_f(
        t + m_h/2,
        vec_add(y, vec_scale(0.5, k2))
    ));

    // k4: slope at the end of the step using k3
    auto k4 = vec_scale(m_h, m_f(
        t + m_h,
        vec_add(y, k3)
    ));

    // Weighted average of the 4 slopes
    // new_y = y + (1/6)*(k1 + 2*k2 + 2*k3 + k4)
    std::vector<double> y_new(y.size());
    for (size_t i = 0; i < y.size(); ++i)
        y_new[i] = y[i] + (1.0/6.0) * (
            k1[i] + 2*k2[i] + 2*k3[i] + k4[i]
        );

    return y_new;
}

// Full solve: repeatedly call step() from t0 to t1
std::vector<std::vector<double>> RK4Solver::solve(
    double t0,
    double t1,
    const std::vector<double>& y0)
{
    std::vector<std::vector<double>> trajectory;
    trajectory.push_back(y0);  // store initial condition

    double t = t0;
    auto y = y0;

    while (t < t1 - 1e-10)  // small tolerance for floating point
    {
        y = step(t, y);
        t += m_h;
        trajectory.push_back(y);
    }

    return trajectory;
}
