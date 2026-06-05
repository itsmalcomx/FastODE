"""
FastODE Interactive Interface (Beta)
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib.pyplot as plt
from fastode_interface import solve


# *********************************************************
# ODE SYSTEM BUILDERS
# ********************************************************

def make_exponential(k=1.0):
    """dy/dt = k*y   (k>0 growth, k<0 decay)."""
    def f(t, y):
        return [k * y[0]]
    return f

def make_logistic(r=0.5, K=100.0):
    """dP/dt = r*P*(1 - P/K)   logistic growth."""
    def f(t, y):
        return [r * y[0] * (1 - y[0] / K)]
    return f

def make_harmonic(omega=1.0):
    """Frictionless oscillation."""
    def f(t, y):
        return [y[1], -omega**2 * y[0]]
    return f

def make_damped(omega=2.0, damping=0.3):
    """Oscillation with friction."""
    def f(t, y):
        return [y[1], -omega**2 * y[0] - 2*damping*y[1]]
    return f

def make_lotka_volterra(a=1.0, b=0.1, d=0.075, g=1.5):
    """Predator-prey."""
    def f(t, y):
        return [a*y[0] - b*y[0]*y[1],
                d*y[0]*y[1] - g*y[1]]
    return f

def make_sir(beta=0.3, gamma=0.1):
    """SIR epidemic."""
    def f(t, y):
        S, I, R = y
        N = S + I + R
        return [-beta*S*I/N,
                beta*S*I/N - gamma*I,
                gamma*I]
    return f

def make_rc_circuit(R=1.0, C=1.0, V_in=5.0):
    """Capacitor charging."""
    def f(t, y):
        return [(V_in - y[0]) / (R * C)]
    return f


# *********************************************************
# SYSTEM REGISTRY
# *********************************************************
SYSTEMS = {
    "1": {
        "name": "Exponential Growth / Decay",
        "desc": "A quantity that changes proportional to itself",
        "builder": make_exponential,
        "params": {
            "k": (0.1, "rate per time unit (positive=growth, negative=decay)"),
        },
        "states": {
            "amount": (100.0, "starting amount"),
        },
        "t1": 10.0,
        "time_unit": "time",
        "labels": ["Amount"],
    },
    "2": {
        "name": "Population Growth (Logistic)",
        "desc": "Population grows then levels off at a carrying capacity",
        "builder": make_logistic,
        "params": {
            "r": (0.5, "growth rate per time unit (e.g. 0.0095 for 0.95%)"),
            "K": (100.0, "carrying capacity (max population)"),
        },
        "states": {
            "population": (5.0, "starting population"),
        },
        "t1": 20.0,
        "time_unit": "years",
        "labels": ["Population"],
    },
    "3": {
        "name": "Harmonic Oscillator (Spring/Pendulum)",
        "desc": "Frictionless oscillation that repeats forever",
        "builder": make_harmonic,
        "params": {
            "omega": (1.0, "angular frequency (2*pi/period)"),
        },
        "states": {
            "position": (1.0, "starting position"),
            "velocity": (0.0, "starting velocity"),
        },
        "t1": 15.0,
        "time_unit": "seconds",
        "labels": ["Position", "Velocity"],
    },
    "4": {
        "name": "Damped Oscillator (Real Spring)",
        "desc": "Oscillation that slowly dies out due to friction",
        "builder": make_damped,
        "params": {
            "omega": (2.0, "angular frequency"),
            "damping": (0.3, "damping factor (friction)"),
        },
        "states": {
            "position": (1.0, "starting position"),
            "velocity": (0.0, "starting velocity"),
        },
        "t1": 15.0,
        "time_unit": "seconds",
        "labels": ["Position", "Velocity"],
    },
    "5": {
        "name": "Predator-Prey (Lotka-Volterra)",
        "desc": "Rabbits and foxes — populations cycle",
        "builder": make_lotka_volterra,
        "params": {
            "a": (1.0,   "prey birth rate"),
            "b": (0.1,   "predation rate"),
            "d": (0.075, "predator growth from eating prey"),
            "g": (1.5,   "predator death rate"),
        },
        "states": {
            "prey":     (10.0, "starting prey population"),
            "predator": (5.0,  "starting predator population"),
        },
        "t1": 40.0,
        "time_unit": "years",
        "labels": ["Prey", "Predator"],
    },
    "6": {
        "name": "Disease Spread (SIR Epidemic)",
        "desc": "How an infection spreads through a population",
        "builder": make_sir,
        "params": {
            "beta":  (0.3, "infection rate per time unit"),
            "gamma": (0.1, "recovery rate (1/days_sick)"),
        },
        "states": {
            "susceptible": (990.0, "people who can catch it"),
            "infected":    (10.0,  "people currently sick"),
            "recovered":   (0.0,   "people recovered/immune"),
        },
        "t1": 100.0,
        "time_unit": "days",
        "labels": ["Susceptible", "Infected", "Recovered"],
    },
    "7": {
        "name": "RC Circuit (Capacitor Charging)",
        "desc": "Voltage across a charging capacitor",
        "builder": make_rc_circuit,
        "params": {
            "R":    (1.0, "resistance (ohms)"),
            "C":    (1.0, "capacitance (farads)"),
            "V_in": (5.0, "input voltage (volts)"),
        },
        "states": {
            "voltage": (0.0, "starting voltage"),
        },
        "t1": 6.0,
        "time_unit": "seconds",
        "labels": ["Voltage"],
    },
}


# *********************************************************
# INPUT HELPERS
# *********************************************************
def ask_float(prompt, default):
    """Ask for a float; blank input keeps the default."""
    raw = input(f"   {prompt} [default {default}]: ").strip()
    if raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"      invalid — using default {default}")
        return default


def ask_method():
    """Ask the user to pick RK4 or RK45."""
    print("\n   STEP 4 — Choose the solver method\n")
    print("   [1] RK45 — adaptive step size (smart, fewer steps)")
    print("   [2] RK4  — fixed step size (simple, you choose step)")
    choice = input("   Method [default 1]: ").strip()
    if choice == "2":
        h = ask_float("RK4 fixed step size h", 0.1)
        return "RK4", h
    return "RK45", None


def print_menu():
    print("\n" + "=" * 60)
    print("   FastODE Interactive Interface  (Beta)")
    print("=" * 60)
    print("\n   Choose a real-world system to simulate:\n")
    for key, s in SYSTEMS.items():
        print(f"   [{key}]  {s['name']}")
        print(f"        {s['desc']}")
    print("\n   [q]  Quit")
    print("-" * 60)


# *********************************************************
# RUN ONE SYSTEM — fully interactive
# *********************************************************
def run_system(s):
    print("\n" + "=" * 60)
    print(f"   {s['name']}")
    print("=" * 60)

    # 1. Rate parameters
    print("\n   STEP 1 — Set the rate parameters")
    print("   (press ENTER to accept each default)\n")
    params = {}
    for pname, (pdefault, pdesc) in s["params"].items():
        print(f"   {pname} — {pdesc}")
        params[pname] = ask_float(pname, pdefault)

    # 2. Initial conditions
    print("\n   STEP 2 — Set the starting values")
    print("   (press ENTER to accept each default)\n")
    y0 = []
    for sname, (sdefault, sdesc) in s["states"].items():
        print(f"   {sname} — {sdesc}")
        y0.append(ask_float(sname, sdefault))

    # 3. Time span
    print("\n   STEP 3 — How long to simulate?\n")
    t1 = ask_float(f"time span (in {s['time_unit']})", s["t1"])

    # 4. Method choice (RK4 or RK45)
    method, h = ask_method()

    # Build and solve
    f = s["builder"](**params)
    print(f"\n   Solving with FastODE ({method})...")
    start = time.perf_counter()
    if method == "RK4":
        result = solve(f, t0=0, t1=t1, y0=y0, method="RK4", h=h)
    else:
        result = solve(f, t0=0, t1=t1, y0=y0, method="RK45")
    elapsed = (time.perf_counter() - start) * 1000

    # Report
    print(f"   Done in {elapsed:.2f} ms")
    print(f"   Steps taken: {result.n_steps}  ·  rejected: {result.n_rejected}")
    print("\n   Results:")
    print(f"      {'Variable':<15}{'Start':>14}{'End':>14}")
    print("      " + "-" * 43)
    for i, label in enumerate(s["labels"]):
        print(f"      {label:<15}{result.y[i,0]:>14.4f}{result.y[i,-1]:>14.4f}")

    plot_result(s, result, t1, method)


# *********************************************************         
# PLOTTING
# *********************************************************
def plot_result(s, result, t1, method="RK45"):
    n_dims = len(s["labels"])

    if n_dims >= 2:
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle(f"FastODE ({method}) — {s['name']}",
                     fontsize=14, fontweight="bold")

        for i, label in enumerate(s["labels"]):
            axes[0].plot(result.t, result.y[i], linewidth=2, label=label)
        axes[0].set_xlabel(f"Time ({s['time_unit']})")
        axes[0].set_ylabel("Value")
        axes[0].set_title("Time Evolution")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(result.y[0], result.y[1], color="green", linewidth=1.5)
        axes[1].plot(result.y[0, 0], result.y[1, 0], "go",
                     markersize=10, label="Start")
        axes[1].set_xlabel(s["labels"][0])
        axes[1].set_ylabel(s["labels"][1])
        axes[1].set_title("Phase Portrait")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
    else:
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(result.t, result.y[0], linewidth=2,
                color="steelblue", label=s["labels"][0])
        ax.set_xlabel(f"Time ({s['time_unit']})")
        ax.set_ylabel(s["labels"][0])
        ax.set_title(f"FastODE ({method}) — {s['name']}", fontweight="bold")
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# *********************************************************         
# MAIN LOOP
# *********************************************************
def main():
    print("""
    Welcome to FastODE — choose any real-world system,
    set your own parameters, starting values, time span,
    and solver method, then watch it solve in milliseconds.
    """)

    while True:
        print_menu()
        choice = input("\n   Your choice: ").strip().lower()

        if choice == "q":
            print("\n   Thanks for using FastODE!\n")
            break
        elif choice in SYSTEMS:
            run_system(SYSTEMS[choice])
            input("\n   [ Press ENTER to return to menu ]")
        else:
            print("\n   Invalid choice, please try again.")


if __name__ == "__main__":
    main()