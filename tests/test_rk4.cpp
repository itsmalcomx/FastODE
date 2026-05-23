#include <gtest/gtest.h>
#include "rk4.hpp"
#include <cmath>

// Test 1: Exponential Decay
// dy/dt = -y,  y(0) = 1
// Exact solution: y(t) = e^(-t)
TEST(RK4Test, ExponentialDecay)
{
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{-y[0]};
    };

    RK4Solver solver(f, 0.01);
    auto trajectory = solver.solve(0.0, 1.0, {1.0});

    // Use back() to get last step
    double computed = trajectory.back()[0];
    double exact    = std::exp(-1.0);

    EXPECT_NEAR(computed, exact, 1e-6);
}

// Test 2: Harmonic Oscillator
// dy1/dt = y2,  dy2/dt = -y1
// Exact: y1 = cos(t), y2 = -sin(t)
TEST(RK4Test, HarmonicOscillator)
{
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{y[1], -y[0]};
    };

    RK4Solver solver(f, 0.001);
    auto trajectory = solver.solve(0.0, 1.0, {1.0, 0.0});

    double y1 = trajectory.back()[0];
    double y2 = trajectory.back()[1];

    EXPECT_NEAR(y1,  std::cos(1.0), 1e-5);
    EXPECT_NEAR(y2, -std::sin(1.0), 1e-5);
}

// Test 3: Constant Solution
// dy/dt = 0 — solution stays constant
TEST(RK4Test, ConstantSolution)
{
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{0.0};
    };

    RK4Solver solver(f, 0.1);
    auto trajectory = solver.solve(0.0, 1.0, {5.0});

    EXPECT_NEAR(trajectory.back()[0], 5.0, 1e-10);
}