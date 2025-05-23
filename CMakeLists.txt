# CMakeLists.txt – Minimal build script for the Visibility-Graph demo
# -------------------------------------------------------------------
# Generates an executable named `demo` that links against Eigen (header-only)
# and the mpocv plotting library used by visualize().
#
# Usage:
#   mkdir build && cd build
#   cmake -DEIGEN3_INCLUDE_DIR=/abs/path/to/eigen \
#         -Dmpocv_DIR=/abs/path/to/mpocv/cmake ..     # if mpocv provides a config
#   cmake --build . --config Release
#
# If mpocv does **not** supply a CMake package, set MPOCV_INCLUDE_DIR and
# MPOCV_LIBS manually, e.g.:
#   cmake -DMPOCV_INCLUDE_DIR=/p/to/mpocv/include \
#         -DMPOCV_LIBS=/p/to/mpocv/lib/libmpocv.so ..
# -------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(VisibilityGraphDemo LANGUAGES CXX)

# ---------- C++ settings -----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- Eigen (header-only) ----------------------------------------------
set(EIGEN3_INCLUDE_DIR "C:/eigen")
message(STATUS "Eigen include dir: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

# ---------- mpocv plotting library -------------------------------------------
add_subdirectory(
    "C:/Users/mwhan/Documents/CODE_SANDBOX/CPlusPlus/Solutions/MatPlotOpenCV/MatPlotOpenCV"
    "${CMAKE_BINARY_DIR}/external/mpocv"
)

# ---------- demo executable ---------------------------------------------------
add_executable(demo
    main_demo.cpp
)

target_link_libraries(demo PRIVATE mpocv)
