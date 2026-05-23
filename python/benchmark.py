"""
benchmark.py
Compare FastODE performance against SciPy and pure Python RK4.
"""

import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))
sys.path.insert(0, os.path.dirname(__file__))

from fastode_interface import solve
from scipy.integrate import solve_ivp


def benchmark(name, f, t0, t1, y0, n_runs=100):
    """Run each solver n_runs times and return average time."""
    print(f"\n{'='*55}")
    print(f"Benchmark: {name}")
    print(f"{'='*55}")

    # FastODE RK45
    start = time.perf_counter()
    for _ in range(n_runs):
        result = solve(f, t0=t0, t1=t1, y0=y0, method='RK45')
    fastode_time = (time.perf_counter() - start) / n_runs

    # SciPy RK45
    start = time.perf_counter()
    for _ in range(n_runs):
        sci = solve_ivp(f, [t0, t1], y0, method='RK45',
                        rtol=1e-6, atol=1e-9)
    scipy_time = (time.perf_counter() - start) / n_runs

    # Results
    speedup = scipy_time / fastode_time
    print(f"FastODE avg time: {fastode_time*1000:.3f} ms")
    print(f"SciPy   avg time: {scipy_time*1000:.3f} ms")
    print(f"Speedup:          {speedup:.2f}x "
          f"({'faster' if speedup > 1 else 'slower'})")
    print(f"FastODE steps:    {result.n_steps}")
    print(f"SciPy steps:      {sci.t.shape[0]}")

    return fastode_time, scipy_time, speedup


# ODE problems
def exp_decay(t, y):
    return [-y[0]]

def harmonic(t, y):
    return [y[1], -y[0]]

def lotka_volterra(t, y):
    alpha, beta, delta, gamma = 1.0, 0.1, 0.075, 1.5
    return [
        alpha*y[0] - beta*y[0]*y[1],
        delta*y[0]*y[1] - gamma*y[1]
    ]


if __name__ == "__main__":
    print("FastODE vs SciPy Benchmark")
    print(f"Each solver run {100} times, average taken\n")

    results = []
    results.append(benchmark("Exponential Decay",
                              exp_decay, 0, 5, [1.0]))
    results.append(benchmark("Harmonic Oscillator",
                              harmonic, 0, 5, [1.0, 0.0]))
    results.append(benchmark("Lotka-Volterra",
                              lotka_volterra, 0, 10, [10.0, 5.0]))

    # Summary
    print(f"\n{'='*55}")
    print("Summary")
    print(f"{'='*55}")
    names = ["Exponential Decay", "Harmonic Oscillator",
             "Lotka-Volterra"]
    for name, (ft, st, sp) in zip(names, results):
        status = "faster" if sp > 1 else "slower"
        print(f"{name:25s}: {sp:.2f}x {status}")