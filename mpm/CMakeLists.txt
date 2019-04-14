set(LIBRARY_NAME splash_lib)

file(GLOB_RECURSE SRCFILES src/*.cpp)

if(NOT SPLASH_WITH_OPENMP)
    list(REMOVE_ITEM SRCFILES "solver/ompSolver.cpp")
endif()

add_library(${LIBRARY_NAME} SHARED ${SRCFILES})


target_include_directories(${LIBRARY_NAME} 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIBRARY_NAME}   ${splash_depends})
                               