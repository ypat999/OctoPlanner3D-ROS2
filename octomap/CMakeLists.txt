
add_library(Octomap STATIC src/pcd2octomap_converter.cpp)

set_target_properties(Octomap PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(Octomap
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(Octomap PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(Octomap
  PUBLIC
    ${PCL_LIBRARIES}
    octomap
    octomath
)
