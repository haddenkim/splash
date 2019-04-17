cmake_minimum_required(VERSION 3.9)

# option(SPLASH_WITH_OPENMP  "Build OpenMP Solver"    ON) 
# option(SPLASH_WITH_MPI     "Build MPI Solver"       OFF)

add_subdirectory(lib)
add_subdirectory(mpm)
add_subdirectory(programs)
