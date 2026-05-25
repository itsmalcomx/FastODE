"""
fastode_interface.py
Clean Python interface for FastODE — similar to scipy.integrate.solve_ivp
"""

import os
import sys

# Fix DLL loading on Windows
if sys.platform == "win32":
    os.add_dll_directory("C:/Program Files/Python314")
    os.add_dll_directory("C:/ProgramData/mingw64/mingw64/bin")

import numpy as np

# Import the C++ bindings
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../build"))
# from fastode import RK4Solver, RK45Solver

# Import the C++ bindings
import os
_here = os.path.dirname(os.path.abspath(__file__))
_build = os.path.join(_here, "..", "build")
sys.path.insert(0, os.path.abspath(_build))
from fastode import RK4Solver, RK45Solver


class ODEResult:
    """
    Stores the result of an ODE solve.
    Similar to scipy's OdeSolution object.
    """
    def __init__(self, t, y, n_steps, n_rejected, method):
        self.t          = np.array(t)
        self.y          = np.array(y)  # already (n_dims, n_steps)
        self.n_steps    = n_steps
        self.n_rejected = n_rejected
        self.method     = method

    # def __init__(self, t, y, n_steps, n_rejected, method):
    #     self.t          = np.array(t)           # 1D array of time points
    #     self.y          = np.array(y).T         # 2D array (n_dims x n_steps)
    #     self.n_steps    = n_steps
    #     self.n_rejected = n_rejected
    #     self.method     = method

    def __repr__(self):
        return (
            f"ODEResult(\n"
            f"  method    = {self.method}\n"
            f"  t         = [{self.t[0]:.3f} ... {self.t[-1]:.3f}]"
            f" ({len(self.t)} points)\n"
            f"  y.shape   = {self.y.shape}\n"
            f"  n_steps   = {self.n_steps}\n"
            f"  n_rejected= {self.n_rejected}\n"
            f")"
        )


def solve(f, t0, t1, y0,
          method  = "RK45",
          h       = 0.01,
          rtol    = 1e-6,
          atol    = 1e-9,
          h_init  = 0.1,
          h_max   = 1.0,
          h_min   = 1e-10):
    """
    Solve an ODE system from t0 to t1.

    Parameters
    ----------
    f      : callable f(t, y) -> list
             The ODE function. Takes time t and state y,
             returns the derivative dy/dt.
    t0     : float — start time
    t1     : float — end time
    y0     : list  — initial condition
    method : str   — 'RK4' or 'RK45' (default 'RK45')
    h      : float — step size for RK4 (default 0.01)
    rtol   : float — relative tolerance for RK45 (default 1e-6)
    atol   : float — absolute tolerance for RK45 (default 1e-9)

    Returns
    -------
    ODEResult with fields: t, y, n_steps, n_rejected, method
    """

    # Validate inputs
    if t1 <= t0:
        raise ValueError("t1 must be greater than t0")
    if len(y0) == 0:
        raise ValueError("y0 must not be empty")
    if method not in ("RK4", "RK45"):
        raise ValueError("method must be 'RK4' or 'RK45'")
    
    if method == "RK4":
        solver = RK4Solver(f, h)
        traj   = solver.solve(t0, t1, y0)
        # Use NumPy buffer protocol — zero copy!
        y_array = np.array(traj, copy=False)
        n_steps    = traj.n_steps() - 1
        n_rejected = 0
        times = list(np.linspace(t0, t1, traj.n_steps()))

    else:  # RK45
        solver = RK45Solver(f, rtol, atol, h_init, h_max, h_min)
        traj   = solver.solve(t0, t1, y0)
        # Use NumPy buffer protocol — zero copy!
        y_array    = np.array(traj, copy=False)
        n_steps    = solver.get_n_steps()
        n_rejected = solver.get_n_rejected()
        times      = solver.get_times()

    # y_array shape is (n_steps, n_dims)
    # transpose to (n_dims, n_steps) for consistency with SciPy
    return ODEResult(times, y_array.T, n_steps, n_rejected, method)
    
    # if method == "RK4":
    #     solver     = RK4Solver(f, h)
    #     traj       = solver.solve(t0, t1, y0)
    #     trajectory = traj.to_nested()
    #     n_steps    = traj.n_steps() - 1
    #     n_rejected = 0
    #     times = list(np.arange(t0, t1 + h, h)[:traj.n_steps()])

    # else:
    #     solver     = RK45Solver(f, rtol, atol, h_init, h_max, h_min)
    #     traj       = solver.solve(t0, t1, y0)
    #     trajectory = traj.to_nested()
    #     n_steps    = solver.get_n_steps()
    #     n_rejected = solver.get_n_rejected()
    #     times      = solver.get_times()

    # return ODEResult(times, trajectory, n_steps, n_rejected, method)

    # if method == "RK4":
    #     solver     = RK4Solver(f, h)
    #     trajectory = solver.solve(t0, t1, y0)
    #     n_steps    = len(trajectory) - 1
    #     n_rejected = 0
    #     # RK4 has uniform time points
    #     import numpy as np
    #     times = list(np.arange(t0, t1 + h, h)[:len(trajectory)])

    # else:  # RK45
    #     solver     = RK45Solver(f, rtol, atol, h_init, h_max, h_min)
    #     trajectory = solver.solve(t0, t1, y0)
    #     n_steps    = solver.get_n_steps()
    #     n_rejected = solver.get_n_rejected()
    #     times      = solver.get_times()

    # return ODEResult(times, trajectory, n_steps, n_rejected, method)
    