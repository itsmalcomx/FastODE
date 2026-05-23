# FastODE Makefile
# Provides simple commands for building and testing
# Works on Linux/Mac. On Windows use build.sh or cmake directly.

PYBIND11_DIR := $(shell python3 -c "import pybind11; print(pybind11.get_cmake_dir())" 2>/dev/null)

.PHONY: all build test validate benchmark clean

all: build

build:
	@echo "Building FastODE..."
	mkdir -p build && cd build && \
	cmake .. -Dpybind11_DIR=$(PYBIND11_DIR) && \
	make

test: build
	@echo "Running C++ unit tests..."
	cd build && ./test_rk4

validate: build
	@echo "Running Python validation vs SciPy..."
	cd build && python3 ../python/validate.py

benchmark: build
	@echo "Running performance benchmark..."
	cd build && python3 ../python/benchmark.py

clean:
	@echo "Cleaning build directory..."
	rm -rf build/