set(LIBRARY_NAME mpm)

file(GLOB_RECURSE SRCFILES src/*.cpp)
add_library(${LIBRARY_NAME} SHARED ${SRCFILES})

target_include_directories(${LIBRARY_NAME} 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIBRARY_NAME}   igl::core 
                                        igl::opengl_glfw  
                                        igl::opengl_glfw_imgui)
                                        
                                        