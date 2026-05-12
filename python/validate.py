"""
validate.py
Compare FastODE results against scipy.integrate.solve_ivp
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))
sys.path.insert(0, os.path.dirname(__file__))

from fastode_interface import solve
from scipy.integrate import solve_ivp


def compare(name, f, t0, t1, y0, rtol=1e-5):
    print(f"\n{'='*50}")
    print(f"Test: {name}")
    print(f"{'='*50}")

    fast = solve(f, t0=t0, t1=t1, y0=y0, method='RK45')
    sci  = solve_ivp(f, [t0, t1], y0, method='RK45',
                     rtol=1e-6, atol=1e-9, dense_output=False)

    fast_final = fast.y[0, -1]
    sci_final  = sci.y[0, -1]

    print(f"FastODE final:  {fast_final:.8f}")
    print(f"SciPy final:    {sci_final:.8f}")
    print(f"Difference:     {abs(fast_final - sci_final):.2e}")
    print(f"FastODE steps:  {fast.n_steps}")
    print(f"SciPy steps:    {sci.t.shape[0]}")

    match = abs(fast_final - sci_final) < rtol
    print(f"Match (rtol={rtol}): {'PASS' if match else 'FAIL'}")
    return match


# Test 1: Exponential Decay
def exp_decay(t, y):
    return [-y[0]]

# Test 2: Harmonic Oscillator
def harmonic(t, y):
    return [y[1], -y[0]]

# Test 3: Lotka-Volterra
def lotka_volterra(t, y):
    alpha, beta, delta, gamma = 1.0, 0.1, 0.075, 1.5
    return [
        alpha*y[0] - beta*y[0]*y[1],
        delta*y[0]*y[1] - gamma*y[1]
    ]


if __name__ == "__main__":
    results = []
    results.append(compare("Exponential Decay",   exp_decay,      0, 5,  [1.0]))
    results.append(compare("Harmonic Oscillator", harmonic,       0, 5,  [1.0, 0.0]))
    results.append(compare("Lotka-Volterra",      lotka_volterra, 0, 10, [10.0, 5.0]))

    all_pass = all(results)
    print(f"\n{'='*50}")
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")
    print(f"{'='*50}")