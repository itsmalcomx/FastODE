#include "rk4.hpp"
#include <cmath>
#include <stdexcept>

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

// ─────────────────────────────────────────────
// Dormand-Prince RK45 Butcher tableau coefficients
// ─────────────────────────────────────────────
// These are fixed constants defined by Dormand & Prince (1980)
// They define exactly how the 6 stages are computed

// c coefficients (time offsets for each stage)
static const double c2 = 1.0/5.0;
static const double c3 = 3.0/10.0;
static const double c4 = 4.0/5.0;
static const double c5 = 8.0/9.0;
// c6 = 1.0, c7 = 1.0

// a coefficients (how stages depend on previous stages)
static const double a21 = 1.0/5.0;
static const double a31 = 3.0/40.0,      a32 = 9.0/40.0;
static const double a41 = 44.0/45.0,     a42 = -56.0/15.0,    a43 = 32.0/9.0;
static const double a51 = 19372.0/6561.0,a52 = -25360.0/2187.0,
                    a53 = 64448.0/6561.0, a54 = -212.0/729.0;
static const double a61 = 9017.0/3168.0, a62 = -355.0/33.0,
                    a63 = 46732.0/5247.0, a64 = 49.0/176.0,
                    a65 = -5103.0/18656.0;

// b coefficients for 5th order solution
static const double b1 = 35.0/384.0,   b3 = 500.0/1113.0,
                    b4 = 125.0/192.0,   b5 = -2187.0/6784.0,
                    b6 = 11.0/84.0;

// e coefficients for error estimate (difference between 4th and 5th order)
static const double e1 =  71.0/57600.0,  e3 = -71.0/16695.0,
                    e4 =  71.0/1920.0,   e5 = -17253.0/339200.0,
                    e6 =  22.0/525.0,    e7 = -1.0/40.0;

// ─────────────────────────────────────────────
// RK45Solver constructor
// ─────────────────────────────────────────────
RK45Solver::RK45Solver(ODEFunc f,
                       double rtol,
                       double atol,
                       double h_init,
                       double h_max,
                       double h_min)
    : m_f(f)
    , m_rtol(rtol)
    , m_atol(atol)
    , m_h_init(h_init)
    , m_h_max(h_max)
    , m_h_min(h_min)
    , m_n_steps(0)
    , m_n_rejected(0)
{}

// ─────────────────────────────────────────────
// Single RK45 step using Dormand-Prince
// Returns: {new_y, error_norm}
// ─────────────────────────────────────────────
std::pair<std::vector<double>, double>
RK45Solver::step(double t,
                 const std::vector<double>& y,
                 double h)
{
    size_t n = y.size();

    // Compute the 6 stages k1..k6
    // Each stage is h * f(t + c*h, y + a*k_prev)
    auto k1 = vec_scale(h, m_f(t, y));

    auto k2 = vec_scale(h, m_f(t + c2*h,
        vec_add(y, vec_scale(a21, k1))));

    auto k3 = vec_scale(h, m_f(t + c3*h,
        vec_add(y, vec_add(
            vec_scale(a31, k1),
            vec_scale(a32, k2)))));

    auto k4 = vec_scale(h, m_f(t + c4*h,
        vec_add(y, vec_add(vec_add(
            vec_scale(a41, k1),
            vec_scale(a42, k2)),
            vec_scale(a43, k3)))));

    auto k5 = vec_scale(h, m_f(t + c5*h,
        vec_add(y, vec_add(vec_add(vec_add(
            vec_scale(a51, k1),
            vec_scale(a52, k2)),
            vec_scale(a53, k3)),
            vec_scale(a54, k4)))));

    auto k6 = vec_scale(h, m_f(t + h,
        vec_add(y, vec_add(vec_add(vec_add(vec_add(
            vec_scale(a61, k1),
            vec_scale(a62, k2)),
            vec_scale(a63, k3)),
            vec_scale(a64, k4)),
            vec_scale(a65, k5)))));

    // 5th order solution
    std::vector<double> y_new(n);
    for (size_t i = 0; i < n; ++i)
        y_new[i] = y[i] + b1*k1[i] + b3*k3[i]
                        + b4*k4[i] + b5*k5[i] + b6*k6[i];

    // 7th stage (needed for error estimate)
    auto k7 = vec_scale(h, m_f(t + h, y_new));

    // Error estimate: difference between 4th and 5th order
    std::vector<double> err(n);
    for (size_t i = 0; i < n; ++i)
        err[i] = e1*k1[i] + e3*k3[i] + e4*k4[i]
               + e5*k5[i] + e6*k6[i] + e7*k7[i];

    // Compute scalar error norm
    // norm = sqrt( mean( (err / (atol + rtol*|y|))^2 ) )
    double err_norm = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        double scale = m_atol + m_rtol * std::max(
            std::abs(y[i]), std::abs(y_new[i])
        );
        err_norm += (err[i] / scale) * (err[i] / scale);
    }
    err_norm = std::sqrt(err_norm / n);

    return {y_new, err_norm};
}

// ─────────────────────────────────────────────
// Full RK45 solve with adaptive step size
// ─────────────────────────────────────────────
std::vector<std::vector<double>>
RK45Solver::solve(double t0,
                  double t1,
                  const std::vector<double>& y0)
{
    m_n_steps    = 0;
    m_n_rejected = 0;
    m_times.clear();

    std::vector<std::vector<double>> trajectory;
    trajectory.push_back(y0);
    m_times.push_back(t0);

    double t = t0;
    auto   y = y0;
    double h = std::min(m_h_init, t1 - t0);

    while (t < t1 - 1e-10)
    {
        // Don't overshoot the end
        if (t + h > t1) h = t1 - t;

        // Take a step and get error estimate
        auto [y_new, err_norm] = step(t, y, h);

        if (err_norm <= 1.0)
        {
            // Step accepted!
            t += h;
            y  = y_new;
            trajectory.push_back(y);
            m_times.push_back(t);
            m_n_steps++;

            // Increase step size for next step
            // factor = 0.9 * (1/err)^(1/5)
            double factor = 0.9 * std::pow(1.0 / err_norm, 0.2);
            factor = std::min(factor, 10.0);  // max 10x increase
            h = std::min(h * factor, m_h_max);
        }
        else
        {
            // Step rejected — too much error, reduce step size
            m_n_rejected++;
            double factor = 0.9 * std::pow(1.0 / err_norm, 0.2);
            factor = std::max(factor, 0.1);  // max 10x decrease
            h = h * factor;

            if (h < m_h_min)
                throw std::runtime_error(
                    "RK45: step size too small"
                );
        }
    }

    return trajectory;
}
