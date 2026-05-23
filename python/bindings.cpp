#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include "rk4.hpp"
#include <memory>

namespace py = pybind11;

PYBIND11_MODULE(fastode, m)
{
    m.doc() = "FastODE: A lightweight ODE solver in C++ with Python interface";

    // Expose Trajectory class
    py::class_<Trajectory>(m, "Trajectory")
        .def("n_dims",   &Trajectory::n_dims)
        .def("n_steps",  &Trajectory::n_steps)
        .def("get_step", &Trajectory::get_step)
        .def("to_nested",&Trajectory::to_nested);


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

    py::class_<RK45Solver>(m, "RK45Solver")
    .def(py::init<ODEFunc, double, double, double, double, double>(),
         py::arg("f"),
         py::arg("rtol")   = 1e-3,
         py::arg("atol")   = 1e-6,
         py::arg("h_init") = 0.1,
         py::arg("h_max")  = 1.0,
         py::arg("h_min")  = 1e-10,
         "Create an RK45 adaptive solver.")
    .def("solve", &RK45Solver::solve,
         py::arg("t0"),
         py::arg("t1"),
         py::arg("y0"),
         "Solve from t0 to t1 with initial condition y0")
    .def("get_times",      &RK45Solver::get_times)
    .def("get_n_steps",    &RK45Solver::get_n_steps)
    .def("get_n_rejected", &RK45Solver::get_n_rejected);

    // Smart pointer factory function
// unique_ptr ensures solver is automatically destroyed
// when it goes out of scope — no manual memory management
m.def("make_rk45_solver",
    [](ODEFunc f, double rtol, double atol) {
        return std::make_unique<RK45Solver>(f, rtol, atol);
    },
    py::arg("f"),
    py::arg("rtol") = 1e-6,
    py::arg("atol") = 1e-9,
    "Create an RK45Solver using unique_ptr (automatic memory management)"
);
}

