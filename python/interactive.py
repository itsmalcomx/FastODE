"""
interactive.py — FastODE Interactive Interface (Beta)
Run with:  python python/interactive.py

A menu-driven interface that lets the user choose from a library
of real-world ODE systems, set their own parameters, solve, and plot.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt
from fastode_interface import solve


# ═════════════════════════════════════════════════════════
# LIBRARY OF REAL-WORLD ODE SYSTEMS
# Each entry defines: the ODE function, default params,
# initial conditions, time span, and plot labels.
# ═════════════════════════════════════════════════════════

def make_exponential_decay(k=1.0):
    """Radioactive decay / drug metabolism / cooling."""
    def f(t, y):
        return [-k * y[0]]
    return f

def make_population_growth(r=0.5, K=100.0):
    """Logistic population growth with carrying capacity K."""
    def f(t, y):
        return [r * y[0] * (1 - y[0] / K)]
    return f

def make_harmonic(omega=1.0):
    """Spring / pendulum oscillation."""
    def f(t, y):
        return [y[1], -omega**2 * y[0]]
    return f

def make_damped_oscillator(omega=2.0, damping=0.3):
    """Real spring with friction — oscillation that decays."""
    def f(t, y):
        return [y[1], -omega**2 * y[0] - 2*damping*y[1]]
    return f

def make_lotka_volterra(a=1.0, b=0.1, d=0.075, g=1.5):
    """Predator-prey population dynamics."""
    def f(t, y):
        return [a*y[0] - b*y[0]*y[1],
                d*y[0]*y[1] - g*y[1]]
    return f

def make_sir_epidemic(beta=0.3, gamma=0.1):
    """SIR disease spread model (Susceptible-Infected-Recovered)."""
    def f(t, y):
        S, I, R = y
        N = S + I + R
        return [-beta*S*I/N,
                beta*S*I/N - gamma*I,
                gamma*I]
    return f

def make_rc_circuit(R=1.0, C=1.0, V_in=5.0):
    """Capacitor charging in an RC electrical circuit."""
    def f(t, y):
        return [(V_in - y[0]) / (R * C)]
    return f


# ═════════════════════════════════════════════════════════
# SYSTEM REGISTRY — the menu the user picks from
# ═════════════════════════════════════════════════════════
SYSTEMS = {
    "1": {
        "name": "Radioactive Decay / Cooling",
        "desc": "A quantity that decreases proportional to itself",
        "builder": make_exponential_decay,
        "params": {"k": 1.0},
        "y0": [100.0],
        "t1": 5.0,
        "labels": ["Amount"],
    },
    "2": {
        "name": "Population Growth (Logistic)",
        "desc": "Population grows then levels off at carrying capacity",
        "builder": make_population_growth,
        "params": {"r": 0.5, "K": 100.0},
        "y0": [5.0],
        "t1": 20.0,
        "labels": ["Population"],
    },
    "3": {
        "name": "Harmonic Oscillator (Spring/Pendulum)",
        "desc": "Frictionless oscillation — repeats forever",
        "builder": make_harmonic,
        "params": {"omega": 1.0},
        "y0": [1.0, 0.0],
        "t1": 15.0,
        "labels": ["Position", "Velocity"],
    },
    "4": {
        "name": "Damped Oscillator (Real Spring)",
        "desc": "Oscillation that slowly dies out due to friction",
        "builder": make_damped_oscillator,
        "params": {"omega": 2.0, "damping": 0.3},
        "y0": [1.0, 0.0],
        "t1": 15.0,
        "labels": ["Position", "Velocity"],
    },
    "5": {
        "name": "Predator-Prey (Lotka-Volterra)",
        "desc": "Rabbits and foxes — populations cycle",
        "builder": make_lotka_volterra,
        "params": {"a": 1.0, "b": 0.1, "d": 0.075, "g": 1.5},
        "y0": [10.0, 5.0],
        "t1": 40.0,
        "labels": ["Prey", "Predator"],
    },
    "6": {
        "name": "Disease Spread (SIR Epidemic)",
        "desc": "How an infection spreads through a population",
        "builder": make_sir_epidemic,
        "params": {"beta": 0.3, "gamma": 0.1},
        "y0": [990.0, 10.0, 0.0],
        "t1": 100.0,
        "labels": ["Susceptible", "Infected", "Recovered"],
    },
    "7": {
        "name": "RC Circuit (Capacitor Charging)",
        "desc": "Voltage across a charging capacitor",
        "builder": make_rc_circuit,
        "params": {"R": 1.0, "C": 1.0, "V_in": 5.0},
        "y0": [0.0],
        "t1": 6.0,
        "labels": ["Voltage"],
    },
}


# ═════════════════════════════════════════════════════════
# UI HELPERS
# ═════════════════════════════════════════════════════════
def print_menu():
    print("\n" + "=" * 58)
    print("   FastODE Interactive Interface  (Beta)")
    print("=" * 58)
    print("\n   Choose a real-world system to simulate:\n")
    for key, sys_def in SYSTEMS.items():
        print(f"   [{key}]  {sys_def['name']}")
        print(f"        {sys_def['desc']}")
    print("\n   [q]  Quit")
    print("-" * 58)


def ask_parameters(sys_def):
    """Let the user override default parameters, or accept defaults."""
    print(f"\n   Default parameters: {sys_def['params']}")
    choice = input("   Use defaults? [Y/n]: ").strip().lower()

    params = dict(sys_def['params'])
    if choice == 'n':
        for pname, pdefault in sys_def['params'].items():
            val = input(f"   {pname} (default {pdefault}): ").strip()
            if val:
                try:
                    params[pname] = float(val)
                except ValueError:
                    print(f"   Invalid, using default {pdefault}")
    return params


def run_system(sys_def):
    params = ask_parameters(sys_def)

    # Build the ODE function with chosen parameters
    f = sys_def['builder'](**params)

    # Solve
    print("\n   Solving with FastODE (RK45)...")
    start = time.perf_counter()
    result = solve(f, t0=0, t1=sys_def['t1'],
                   y0=sys_def['y0'], method='RK45')
    elapsed = (time.perf_counter() - start) * 1000

    print(f"   Done in {elapsed:.2f} ms")
    print(f"   Steps: {result.n_steps}  ·  Rejected: {result.n_rejected}")

    # Print final values
    print("\n   Final values:")
    for label, val in zip(sys_def['labels'], result.y[:, -1]):
        print(f"      {label:<15} = {val:.4f}")

    # Plot
    n_dims = len(sys_def['labels'])
    if n_dims >= 2:
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle(f"FastODE — {sys_def['name']}",
                     fontsize=14, fontweight='bold')

        # Time series
        for i, label in enumerate(sys_def['labels']):
            axes[0].plot(result.t, result.y[i], linewidth=2, label=label)
        axes[0].set_xlabel('Time')
        axes[0].set_ylabel('Value')
        axes[0].set_title('Time Evolution')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Phase portrait (first two dimensions)
        axes[1].plot(result.y[0], result.y[1], color='green', linewidth=1.5)
        axes[1].plot(result.y[0, 0], result.y[1, 0], 'go',
                     markersize=10, label='Start')
        axes[1].set_xlabel(sys_def['labels'][0])
        axes[1].set_ylabel(sys_def['labels'][1])
        axes[1].set_title('Phase Portrait')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
    else:
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(result.t, result.y[0], linewidth=2,
                color='steelblue', label=sys_def['labels'][0])
        ax.set_xlabel('Time')
        ax.set_ylabel(sys_def['labels'][0])
        ax.set_title(f"FastODE — {sys_def['name']}", fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ═════════════════════════════════════════════════════════
# MAIN LOOP
# ═════════════════════════════════════════════════════════
def main():
    print("""
    Welcome to FastODE — choose any real-world system,
    set your own parameters, and watch it solve in milliseconds.
    """)

    while True:
        print_menu()
        choice = input("\n   Your choice: ").strip().lower()

        if choice == 'q':
            print("\n   Thanks for using FastODE!\n")
            break
        elif choice in SYSTEMS:
            run_system(SYSTEMS[choice])
            input("\n   [ Press ENTER to return to menu ]")
        else:
            print("\n   Invalid choice, please try again.")


if __name__ == "__main__":
    main()