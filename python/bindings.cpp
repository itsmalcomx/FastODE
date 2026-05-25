#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <memory>
#include "rk4.hpp"

namespace py = pybind11;

PYBIND11_MODULE(fastode, m)
{
    m.doc() = "FastODE: lightweight ODE solver in C++ "
              "with Python interface";

    // ─────────────────────────────────────────────
    // Trajectory class
    // Exposes the flat contiguous array to Python
    // using NumPy buffer protocol — ZERO COPY
    //
    // BEFORE: to_nested() copied data twice
    //   C++ vector → Python list → NumPy array
    //
    // AFTER: buffer protocol exposes C++ memory directly
    //   C++ vector → NumPy array (same memory, no copy)
    // ─────────────────────────────────────────────
    py::class_<Trajectory>(m, "Trajectory",
                            py::buffer_protocol())
        .def("n_dims",    &Trajectory::n_dims)
        .def("n_steps",   &Trajectory::n_steps)
        .def("get_step",  &Trajectory::get_step)
        .def("to_nested", &Trajectory::to_nested)
        .def("back",      &Trajectory::back)

        // Buffer protocol — exposes raw C++ memory to NumPy
        // shape: (n_steps, n_dims)
        // strides: how many bytes to jump per row/column
        .def_buffer([](Trajectory& traj) -> py::buffer_info {
            return py::buffer_info(
                // Pointer to raw data
                traj.data(),
                // Size of each element in bytes
                sizeof(double),
                // NumPy format string for double
                py::format_descriptor<double>::format(),
                // Number of dimensions
                2,
                // Shape: (n_steps, n_dims)
                {traj.n_steps(), traj.n_dims()},
                // Strides in bytes:
                // row stride = n_dims * sizeof(double)
                // col stride = sizeof(double)
                {traj.n_dims() * sizeof(double),
                 sizeof(double)}
            );
        });

    // RK4Solver
    py::class_<RK4Solver>(m, "RK4Solver")
        .def(py::init<ODEFunc, double>(),
             py::arg("f"),
             py::arg("h"),
             "Create RK4 solver with fixed step size h")
        .def("step",  &RK4Solver::step)
        .def("solve", &RK4Solver::solve,
             "Solve ODE from t0 to t1 with initial condition y0");

    // RK45Solver
    py::class_<RK45Solver>(m, "RK45Solver")
        .def(py::init<ODEFunc, double, double,
                      double, double, double>(),
             py::arg("f"),
             py::arg("rtol")   = 1e-3,
             py::arg("atol")   = 1e-6,
             py::arg("h_init") = 0.1,
             py::arg("h_max")  = 1.0,
             py::arg("h_min")  = 1e-10,
             "Create RK45 adaptive solver")
        .def("solve",         &RK45Solver::solve)
        .def("get_times",     &RK45Solver::get_times)
        .def("get_n_steps",   &RK45Solver::get_n_steps)
        .def("get_n_rejected",&RK45Solver::get_n_rejected);

    // Smart pointer factory
    m.def("make_rk45_solver",
        [](ODEFunc f, double rtol, double atol) {
            return std::make_unique<RK45Solver>(f, rtol, atol);
        },
        py::arg("f"),
        py::arg("rtol") = 1e-6,
        py::arg("atol") = 1e-9,
        "Create RK45Solver via unique_ptr"
    );
}