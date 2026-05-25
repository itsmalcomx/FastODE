# FastODE

A lightweight, high-performance ODE solver in C++ with a Python interface.

FastODE implements two industry-standard numerical methods for solving
Ordinary Differential Equations (ODEs), achieving approximately 4x better
performance than SciPy while producing identical results.

# What Is An ODE?

An Ordinary Differential Equation describes how a system changes over time:
    dy/dt = f(t, y),  y(t0) = y0
Real-world examples:
- **Predator-prey dynamics** — rabbit and fox populations
- **Harmonic oscillator** — spring-mass systems, pendulums
- **Exponential decay** — radioactive decay, drug metabolism
- **Electrical circuits** — voltage and current over time

# Performance

Benchmarked over 100 runs each, averaged:

| Problem | FastODE | SciPy | Speedup |
|---------|---------|-------|---------|
| Exponential Decay | 0.30 ms | 1.21 ms | **4.03x** |
| Harmonic Oscillator | 0.40 ms | 1.89 ms | **4.72x** |
| Lotka-Volterra | 0.90 ms | 3.53 ms | **3.91x** |

Speedup achieved through:
- C++ numerical core (no Python interpreter overhead)
- Pre-allocated workspace vectors (eliminates ~450 heap allocations per solve)
- Flat contiguous trajectory storage (cache-friendly memory layout)
- NumPy buffer protocol (zero-copy data transfer to Python)
- Compiler optimization (`-O3 -march=native`)

# Correctness

FastODE results validated against SciPy (`scipy.integrate.solve_ivp`):

| Problem | Difference | Status |
|---------|-----------|--------|
| Exponential Decay | 5.98e-13 | ✅ PASS |
| Harmonic Oscillator | 5.20e-10 | ✅ PASS |
| Lotka-Volterra | 3.86e-06 | ✅ PASS |

# Quick Start

```python
from python.fastode_interface import solve
import matplotlib.pyplot as plt

# Predator-prey system (Lotka-Volterra)
def lotka_volterra(t, y):
    return [
        1.0*y[0] - 0.1*y[0]*y[1],   # rabbits
        0.075*y[0]*y[1] - 1.5*y[1]  # foxes
    ]

result = solve(lotka_volterra,
               t0=0, t1=30,
               y0=[10.0, 5.0],
               method='RK45')

plt.plot(result.t, result.y[0], label='Rabbits')
plt.plot(result.t, result.y[1], label='Foxes')
plt.legend()
plt.show()
```

# Installation

# Prerequisites
- CMake 3.18+
- g++ with C++17 support
- Python 3.8+
- pybind11, numpy, scipy

# Build

```bash
# Automated setup
bash scripts/setup.sh

# Or manually
make build
```

# Test

```bash
make test       # C++ unit tests
make validate   # Python validation vs SciPy
make benchmark  # Performance benchmark
```

# Project Structure

FastODE/
include/
rk4.hpp          # RK4 + RK45 class declarations
src/
rk4.cpp          # RK4 + RK45 implementations
tests/
test_rk4.cpp     # Google Test unit tests
python/
bindings.cpp     # pybind11 C++ bindings
fastode_interface.py  # Python API
validate.py      # Validation vs SciPy
benchmark.py     # Performance benchmarking
scripts/
setup.sh         # Automated build setup
validate_ci.py   # CI validation script
.github/
workflows/
ci.yml         # GitHub Actions CI
Makefile           # Build automation

# Algorithms

# RK4 (Fixed Step)
Classical 4th order Runge-Kutta. Uses 4 slope estimates per step
combined with a weighted average. Fixed step size `h` chosen by user.

# RK45 (Dormand-Prince Adaptive)
Embedded 4th/5th order method. Computes two solutions simultaneously
and uses their difference as an error estimate to automatically adjust
step size. Achieves same accuracy as RK4 in far fewer steps.

# Engineering Concepts

This project demonstrates concepts from the NSD course:

| Concept | Implementation |
|---------|---------------|
| CMake | Cross-platform build system |
| Google Test | C++ unit testing |
| pytest | Python validation |
| pybind11 | C++/Python interoperability |
| GitHub Actions | Continuous integration |
| Memory Management | Pre-allocated workspace vectors |
| Array Code in C++ | Flat contiguous Trajectory class |
| NumPy Buffer Protocol | Zero-copy C++ to Python data transfer |
| Cache Optimization | -O3 -march=native compiler flags |
| Smart Pointers | unique_ptr factory for solvers |
| Modern C++17 | Structured bindings, lambdas, std::function |

# License
MIT License — see [LICENSE](LICENSE) for details.