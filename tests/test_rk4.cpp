#include <gtest/gtest.h>
#include "rk4.hpp"
#include <cmath>

// Test 1: Exponential Decay
// dy/dt = -y,  y(0) = 1
// Exact solution: y(t) = e^(-t)
// This is the simplest ODE to verify correctness
TEST(RK4Test, ExponentialDecay)
{
    // Define f(t, y) = -y
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{-y[0]};
    };

    // Create solver with step size h = 0.01
    RK4Solver solver(f, 0.01);

    // Solve from t=0 to t=1, starting at y=1
    auto trajectory = solver.solve(0.0, 1.0, {1.0});

    // Last value should be close to e^(-1) = 0.36787...
    double computed = trajectory.back()[0];
    double exact    = std::exp(-1.0);

    // Allow small error (RK4 is very accurate)
    EXPECT_NEAR(computed, exact, 1e-6);
}

// Test 2: Simple Harmonic Oscillator
// dy1/dt =  y2
// dy2/dt = -y1
// This models a spring-mass system
// Exact solution: y1(t) = cos(t), y2(t) = -sin(t)
TEST(RK4Test, HarmonicOscillator)
{
    // Define f(t, y) = [y[1], -y[0]]
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{y[1], -y[0]};
    };

    // Create solver with step size h = 0.001
    RK4Solver solver(f, 0.001);

    // Solve from t=0 to t=1
    // Initial condition: y1=1 (displacement), y2=0 (velocity)
    auto trajectory = solver.solve(0.0, 1.0, {1.0, 0.0});

    // At t=1: y1 should be cos(1) = 0.5403...
    //         y2 should be -sin(1) = -0.8414...
    double y1 = trajectory.back()[0];
    double y2 = trajectory.back()[1];

    EXPECT_NEAR(y1,  std::cos(1.0), 1e-5);
    EXPECT_NEAR(y2, -std::sin(1.0), 1e-5);
}

// Test 3: Zero derivative
// dy/dt = 0,  y(0) = 5
// Solution should stay constant at y = 5
TEST(RK4Test, ConstantSolution)
{
    ODEFunc f = [](double t, const std::vector<double>& y)
    {
        return std::vector<double>{0.0};
    };

    RK4Solver solver(f, 0.1);
    auto trajectory = solver.solve(0.0, 1.0, {5.0});

    // y should still be 5 at the end
    EXPECT_NEAR(trajectory.back()[0], 5.0, 1e-10);
}