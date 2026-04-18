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
