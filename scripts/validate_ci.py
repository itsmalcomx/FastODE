import sys
import os
sys.path.insert(0, '.')
sys.path.insert(0, '../python')
from fastode_interface import solve
import math

def f(t, y):
    return [-y[0]]

result = solve(f, t0=0.0, t1=1.0, y0=[1.0], method='RK45')
error = abs(result.y[0,-1] - math.exp(-1.0))
print(f'Error: {error:.2e}')
assert error < 1e-5, f'Error too large: {error}'
print('Python validation passed!')