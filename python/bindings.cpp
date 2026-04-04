#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include "rk4.hpp"

namespace py = pybind11;

PYBIND11_MODULE(fastode, m)
{
    m.doc() = "FastODE: A lightweight ODE solver in C++ with Python interface";


    //using this to expose the RK4Solver class to Python, allowing users to create instances of RK4Solver and call its methods from Python code.
    py::class_<RK4Solver>(m, "RK4Solver")
        .def(py::init<ODEFunc, double>(),
             py::arg("f"),
             py::arg("h"),
             "Create an RK4 solver.\n"
             "f: function f(t, y) -> y'\n"
             "h: step size")

        .def("step", &RK4Solver::step,
             py::arg("t"),
             py::arg("y"),
             "Take a single RK4 step from time t with state y")

        .def("solve", &RK4Solver::solve,
             py::arg("t0"),
             py::arg("t1"),
             py::arg("y0"),
             "Solve from t0 to t1 with initial condition y0");
}