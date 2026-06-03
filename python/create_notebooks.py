"""
create_notebooks.py
Creates proper Jupyter notebooks for FastODE demos
"""
import nbformat as nbf
import os

os.makedirs("notebooks", exist_ok=True)

# ─────────────────────────────────────────────
# Notebook 1: Exponential Decay
# ─────────────────────────────────────────────
nb1 = nbf.v4.new_notebook()
nb1.cells = [
    nbf.v4.new_markdown_cell(
        "# Exponential Decay\n\n"
        "## What Is It?\n"
        "Exponential decay describes any system where the rate of\n"
        "decrease is proportional to the current value.\n\n"
        "**Real world examples:**\n"
        "- Radioactive decay\n"
        "- Drug leaving your bloodstream\n"
        "- Coffee cooling down\n\n"
        "## The ODE\n"
        "`dy/dt = -y,  y(0) = 1`\n\n"
        "**Exact solution:** `y(t) = e^(-t)`"
    ),
    nbf.v4.new_code_cell(
        "import sys, os\n"
        "sys.path.insert(0, os.path.join('..', 'python'))\n"
        "sys.path.insert(0, os.path.join('..', 'build'))\n"
        "import numpy as np\n"
        "import matplotlib.pyplot as plt\n"
        "from fastode_interface import solve\n"
        "print('FastODE loaded!')"
    ),
    nbf.v4.new_code_cell(
        "def f(t, y):\n"
        "    return [-y[0]]\n\n"
        "result = solve(f, t0=0.0, t1=5.0, y0=[1.0], method='RK45')\n"
        "t_exact = np.linspace(0, 5, 200)\n"
        "y_exact = np.exp(-t_exact)\n\n"
        "print(f'Steps taken:    {result.n_steps}')\n"
        "print(f'Final value:    {result.y[0,-1]:.8f}')\n"
        "print(f'Exact value:    {np.exp(-5):.8f}')\n"
        "print(f'Error:          {abs(result.y[0,-1] - np.exp(-5)):.2e}')"
    ),
    nbf.v4.new_code_cell(
        "fig, axes = plt.subplots(1, 2, figsize=(12, 4))\n\n"
        "axes[0].plot(t_exact, y_exact, 'b-', linewidth=2, label='Exact')\n"
        "axes[0].plot(result.t, result.y[0], 'ro--', markersize=8,\n"
        "             label=f'FastODE ({result.n_steps} steps)')\n"
        "axes[0].set_xlabel('Time')\n"
        "axes[0].set_ylabel('y(t)')\n"
        "axes[0].set_title('Exponential Decay')\n"
        "axes[0].legend()\n"
        "axes[0].grid(True, alpha=0.3)\n\n"
        "y_interp = np.interp(result.t, t_exact, y_exact)\n"
        "error = np.abs(result.y[0] - y_interp)\n"
        "axes[1].semilogy(result.t, error + 1e-20, 'g-o', markersize=6)\n"
        "axes[1].set_xlabel('Time')\n"
        "axes[1].set_ylabel('Absolute Error')\n"
        "axes[1].set_title('Error vs Exact Solution')\n"
        "axes[1].grid(True, alpha=0.3)\n\n"
        "plt.tight_layout()\n"
        "plt.savefig('exponential_decay.png', dpi=100)\n"
        "plt.show()\n"
        "print('Plot saved!')"
    ),
    nbf.v4.new_code_cell(
        "rk4  = solve(f, t0=0, t1=5, y0=[1.0], method='RK4',  h=0.5)\n"
        "rk45 = solve(f, t0=0, t1=5, y0=[1.0], method='RK45')\n\n"
        "plt.figure(figsize=(8, 4))\n"
        "plt.plot(t_exact, y_exact, 'b-', linewidth=2, label='Exact')\n"
        "plt.plot(rk4.t,  rk4.y[0],  'ro-', markersize=10,\n"
        "         label=f'RK4  ({rk4.n_steps} steps, h=0.5)')\n"
        "plt.plot(rk45.t, rk45.y[0], 'gs-', markersize=8,\n"
        "         label=f'RK45 ({rk45.n_steps} steps, adaptive)')\n"
        "plt.xlabel('Time')\n"
        "plt.ylabel('y(t)')\n"
        "plt.title('RK4 vs RK45 Comparison')\n"
        "plt.legend()\n"
        "plt.grid(True, alpha=0.3)\n"
        "plt.tight_layout()\n"
        "plt.show()\n"
        "print(f'RK4  used {rk4.n_steps} steps')\n"
        "print(f'RK45 used {rk45.n_steps} steps')\n"
        "print(f'RK45 is {rk4.n_steps/rk45.n_steps:.1f}x more efficient')"
    ),
]

# ─────────────────────────────────────────────
# Notebook 2: Harmonic Oscillator
# ─────────────────────────────────────────────
nb2 = nbf.v4.new_notebook()
nb2.cells = [
    nbf.v4.new_markdown_cell(
        "# Harmonic Oscillator\n\n"
        "## What Is It?\n"
        "A harmonic oscillator models any system that oscillates\n"
        "around an equilibrium — like a spring, pendulum, or LC circuit.\n\n"
        "## The ODE System\n"
        "`dy1/dt = y2`\n"
        "`dy2/dt = -y1`\n\n"
        "Where y1 = position, y2 = velocity\n\n"
        "**Exact solution:** y1(t) = cos(t), y2(t) = -sin(t)"
    ),
    nbf.v4.new_code_cell(
        "import sys, os\n"
        "sys.path.insert(0, os.path.join('..', 'python'))\n"
        "sys.path.insert(0, os.path.join('..', 'build'))\n"
        "import numpy as np\n"
        "import matplotlib.pyplot as plt\n"
        "from fastode_interface import solve"
    ),
    nbf.v4.new_code_cell(
        "def harmonic(t, y):\n"
        "    return [y[1], -y[0]]\n\n"
        "result = solve(harmonic, t0=0, t1=4*np.pi,\n"
        "               y0=[1.0, 0.0], method='RK45')\n"
        "t_exact = np.linspace(0, 4*np.pi, 500)\n"
        "y1_exact = np.cos(t_exact)\n"
        "y2_exact = -np.sin(t_exact)\n\n"
        "print(f'Steps: {result.n_steps}, Rejected: {result.n_rejected}')"
    ),
    nbf.v4.new_code_cell(
        "fig, axes = plt.subplots(1, 3, figsize=(15, 4))\n\n"
        "axes[0].plot(t_exact, y1_exact, 'b-', label='Exact cos(t)')\n"
        "axes[0].plot(result.t, result.y[0], 'r--', label='FastODE')\n"
        "axes[0].set_title('Position y1(t)')\n"
        "axes[0].set_xlabel('Time')\n"
        "axes[0].legend()\n"
        "axes[0].grid(True, alpha=0.3)\n\n"
        "axes[1].plot(t_exact, y2_exact, 'b-', label='Exact -sin(t)')\n"
        "axes[1].plot(result.t, result.y[1], 'r--', label='FastODE')\n"
        "axes[1].set_title('Velocity y2(t)')\n"
        "axes[1].set_xlabel('Time')\n"
        "axes[1].legend()\n"
        "axes[1].grid(True, alpha=0.3)\n\n"
        "axes[2].plot(y1_exact, y2_exact, 'b-', label='Exact')\n"
        "axes[2].plot(result.y[0], result.y[1], 'r--', label='FastODE')\n"
        "axes[2].set_xlabel('Position y1')\n"
        "axes[2].set_ylabel('Velocity y2')\n"
        "axes[2].set_title('Phase Portrait (should be circle)')\n"
        "axes[2].legend()\n"
        "axes[2].grid(True, alpha=0.3)\n"
        "axes[2].set_aspect('equal')\n\n"
        "plt.tight_layout()\n"
        "plt.savefig('harmonic_oscillator.png', dpi=100)\n"
        "plt.show()"
    ),
]

# ─────────────────────────────────────────────
# Notebook 3: Lotka-Volterra
# ─────────────────────────────────────────────
nb3 = nbf.v4.new_notebook()
nb3.cells = [
    nbf.v4.new_markdown_cell(
        "# Lotka-Volterra: Predator-Prey Dynamics\n\n"
        "## What Is It?\n"
        "The Lotka-Volterra equations model how predator and prey\n"
        "populations interact over time.\n\n"
        "**Real world examples:**\n"
        "- Rabbits and foxes in a forest\n"
        "- Fish and sharks in the ocean\n"
        "- Bacteria and viruses\n\n"
        "## The ODE System\n"
        "`dy1/dt = alpha*y1 - beta*y1*y2  (prey)`\n"
        "`dy2/dt = delta*y1*y2 - gamma*y2  (predator)`\n\n"
        "Where:\n"
        "- alpha = prey birth rate\n"
        "- beta  = predation rate\n"
        "- delta = predator growth from eating prey\n"
        "- gamma = predator death rate"
    ),
    nbf.v4.new_code_cell(
        "import sys, os\n"
        "sys.path.insert(0, os.path.join('..', 'python'))\n"
        "sys.path.insert(0, os.path.join('..', 'build'))\n"
        "import numpy as np\n"
        "import matplotlib.pyplot as plt\n"
        "from fastode_interface import solve"
    ),
    nbf.v4.new_code_cell(
        "alpha, beta, delta, gamma = 1.0, 0.1, 0.075, 1.5\n\n"
        "def lotka_volterra(t, y):\n"
        "    rabbits, foxes = y[0], y[1]\n"
        "    drabbits = alpha*rabbits - beta*rabbits*foxes\n"
        "    dfoxes   = delta*rabbits*foxes - gamma*foxes\n"
        "    return [drabbits, dfoxes]\n\n"
        "result = solve(lotka_volterra, t0=0, t1=40,\n"
        "               y0=[10.0, 5.0], method='RK45')\n\n"
        "print(f'Steps: {result.n_steps}, Rejected: {result.n_rejected}')\n"
        "print(f'Time points: {len(result.t)}')"
    ),
    nbf.v4.new_code_cell(
        "fig, axes = plt.subplots(1, 2, figsize=(12, 5))\n\n"
        "axes[0].plot(result.t, result.y[0],\n"
        "             color='steelblue', linewidth=2, label='Rabbits (prey)')\n"
        "axes[0].plot(result.t, result.y[1],\n"
        "             color='coral', linewidth=2, label='Foxes (predator)')\n"
        "axes[0].set_xlabel('Time (years)')\n"
        "axes[0].set_ylabel('Population')\n"
        "axes[0].set_title('Predator-Prey Population Dynamics')\n"
        "axes[0].legend()\n"
        "axes[0].grid(True, alpha=0.3)\n\n"
        "axes[1].plot(result.y[0], result.y[1], color='green', linewidth=1.5)\n"
        "axes[1].plot(result.y[0,0], result.y[1,0], 'go', markersize=10,\n"
        "             label='Start')\n"
        "axes[1].set_xlabel('Rabbits')\n"
        "axes[1].set_ylabel('Foxes')\n"
        "axes[1].set_title('Phase Portrait (closed orbit = stable cycle)')\n"
        "axes[1].legend()\n"
        "axes[1].grid(True, alpha=0.3)\n\n"
        "plt.tight_layout()\n"
        "plt.savefig('lotka_volterra.png', dpi=100)\n"
        "plt.show()"
    ),
    nbf.v4.new_code_cell(
        "fig, ax = plt.subplots(figsize=(7, 7))\n\n"
        "initial_conditions = [\n"
        "    ([10, 5],  'blue'),\n"
        "    ([15, 5],  'red'),\n"
        "    ([10, 10], 'green'),\n"
        "    ([20, 8],  'purple'),\n"
        "]\n\n"
        "for y0, color in initial_conditions:\n"
        "    r = solve(lotka_volterra, t0=0, t1=40,\n"
        "              y0=y0, method='RK45')\n"
        "    ax.plot(r.y[0], r.y[1], color=color,\n"
        "            alpha=0.7, linewidth=1.5, label=f'Start: {y0}')\n"
        "    ax.plot(r.y[0,0], r.y[1,0], 'o', color=color, markersize=8)\n\n"
        "ax.set_xlabel('Rabbits')\n"
        "ax.set_ylabel('Foxes')\n"
        "ax.set_title('Phase Portraits - Different Initial Conditions')\n"
        "ax.legend()\n"
        "ax.grid(True, alpha=0.3)\n"
        "plt.tight_layout()\n"
        "plt.show()"
    ),
]

# Save all notebooks
with open("notebooks/01_exponential_decay.ipynb", "w", encoding="utf-8") as f:
    nbf.write(nb1, f)

with open("notebooks/02_harmonic_oscillator.ipynb", "w", encoding="utf-8") as f:
    nbf.write(nb2, f)

with open("notebooks/03_lotka_volterra.ipynb", "w", encoding="utf-8") as f:
    nbf.write(nb3, f)

print("All 3 notebooks created successfully!")
print("Run: jupyter notebook")
print("Then open notebooks/ folder")