# 从 OctoMap 源码编译 liboctomath 和 liboctomap 共享库
# 源码位于 submodule: third_party/octomap/octomap/

set(OCTOMAP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/octomap/octomap")

# ---- octomath (数学模块) ----
add_library(octomath SHARED
  ${OCTOMAP_SRC_DIR}/src/math/Pose6D.cpp
  ${OCTOMAP_SRC_DIR}/src/math/Quaternion.cpp
  ${OCTOMAP_SRC_DIR}/src/math/Vector3.cpp
)
target_include_directories(octomath
  PUBLIC
    ${OCTOMAP_SRC_DIR}/include
)
set_target_properties(octomath PROPERTIES
  VERSION 1.10.0
  SOVERSION 1.10
)

# ---- octomap (主库) ----
add_library(octomap SHARED
  ${OCTOMAP_SRC_DIR}/src/AbstractOcTree.cpp
  ${OCTOMAP_SRC_DIR}/src/AbstractOccupancyOcTree.cpp
  ${OCTOMAP_SRC_DIR}/src/ColorOcTree.cpp
  ${OCTOMAP_SRC_DIR}/src/CountingOcTree.cpp
  ${OCTOMAP_SRC_DIR}/src/OcTree.cpp
  ${OCTOMAP_SRC_DIR}/src/OcTreeNode.cpp
  ${OCTOMAP_SRC_DIR}/src/OcTreeStamped.cpp
  ${OCTOMAP_SRC_DIR}/src/Pointcloud.cpp
  ${OCTOMAP_SRC_DIR}/src/ScanGraph.cpp
)
target_include_directories(octomap
  PUBLIC
    ${OCTOMAP_SRC_DIR}/include
)
target_link_libraries(octomap PUBLIC octomath)
set_target_properties(octomap PROPERTIES
  VERSION 1.10.0
  SOVERSION 1.10
)
