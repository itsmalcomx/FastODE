#!/usr/bin/env bash
# setup.sh — automate FastODE build environment setup
# Usage: bash scripts/setup.sh

set -e  # exit on any error

echo "FastODE Setup Script"
echo "===================="

# Check dependencies
echo "Checking dependencies..."
command -v cmake  &>/dev/null || { echo "ERROR: cmake not found"; exit 1; }
command -v g++    &>/dev/null || { echo "ERROR: g++ not found"; exit 1; }
command -v python3 &>/dev/null || { echo "ERROR: python3 not found"; exit 1; }
echo "All dependencies found."

# Get pybind11 cmake dir
PYBIND11_DIR=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())" 2>/dev/null)
if [ -z "$PYBIND11_DIR" ]; then
    echo "pybind11 not found. Installing..."
    pip3 install pybind11 numpy scipy
    PYBIND11_DIR=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())")
fi
echo "pybind11 found at: $PYBIND11_DIR"

# Build
echo ""
echo "Building FastODE..."
mkdir -p build && cd build
cmake .. -Dpybind11_DIR=$PYBIND11_DIR
make
cd ..

echo ""
echo "Build complete!"
echo "Run 'make test'      to run C++ tests"
echo "Run 'make validate'  to validate vs SciPy"
echo "Run 'make benchmark' to run benchmarks"