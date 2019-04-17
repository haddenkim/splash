set(LIBRARY_NAME splash_lib)

file(GLOB_RECURSE SRCFILES src/*.cpp)

add_library(${LIBRARY_NAME} SHARED ${SRCFILES})

target_include_directories(${LIBRARY_NAME} 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# For some reason, finding OpenMP here is needed again
# without it openMP only creates 1 thread
# possibly something to do with the OpenMP target staying local to the lib CMAKE file?
find_package(OpenMP REQUIRED)

target_link_libraries(${LIBRARY_NAME}   ${splash_depends})
                               