add_library(GlobalPlanner STATIC src/global_planner.cpp)

set_target_properties(GlobalPlanner PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(GlobalPlanner
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(GlobalPlanner
  PUBLIC
    octomap
    octomath
    OpenMP::OpenMP_CXX
)

target_compile_options(GlobalPlanner PRIVATE ${OpenMP_CXX_FLAGS})
