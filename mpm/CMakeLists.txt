set(LIBRARY_NAME splash_lib)

file(GLOB_RECURSE SRCFILES src/*.cpp)

add_library(${LIBRARY_NAME} SHARED ${SRCFILES})

target_include_directories(${LIBRARY_NAME} 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

## Dependencies
set(libs )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/lib/cmake)

# Eigen
find_package( Eigen3 REQUIRED )
target_include_directories(${LIBRARY_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR}  )

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            ON)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)
set(libs ${libs}    igl::core 
                    igl::opengl_glfw  
                    igl::opengl_glfw_imgui
                    )

# lodepng
set (LODEPNG_DIR  ${CMAKE_SOURCE_DIR}/lib/lodepng)
add_library(lodepng "${LODEPNG_DIR}/lodepng.cpp")
set(libs ${libs}  lodepng)


# OpenMP
find_package(OpenMP REQUIRED)

# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(libs ${libs} OpenMP::OpenMP_CXX)
# set(libs ${libs} ${OpenMP_CXX_FLAGS})



# MPI
# find_package(MPI REQUIRED)
# set(libs ${libs}  ${MPI_C_LIBRARIES})


target_link_libraries(${LIBRARY_NAME}   ${libs})
