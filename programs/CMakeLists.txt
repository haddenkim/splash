project(splash)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# apps
add_executable(${PROJECT_NAME}_collide collide.cpp)
target_link_libraries(${PROJECT_NAME}_collide splash_lib)

add_executable(${PROJECT_NAME}_fall fall.cpp)
target_link_libraries(${PROJECT_NAME}_fall splash_lib)

add_executable(${PROJECT_NAME}_wall wall.cpp)
target_link_libraries(${PROJECT_NAME}_wall splash_lib)

# benchmarks
add_executable(${PROJECT_NAME}_bench_gather benchGather.cpp)
target_link_libraries(${PROJECT_NAME}_bench_gather splash_lib)

add_executable(${PROJECT_NAME}_bench_atomic benchAtomic.cpp)
target_link_libraries(${PROJECT_NAME}_bench_atomic splash_lib)