
# Utilities
cmake_minimum_required( VERSION 2.8 )
project( pcl_progs )

# PCL
find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS} include/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set include_directories
include_directories(.)

# Set library directories
link_directories( ${CMAKE_SOURCE_DIR}/lib )

# Create library
set( utils_srcs src/mesh2pcd.cpp )
set( utils_hdrs src/getBoundingBox.h src/getBoundingBox.hpp src/mesh2pcd.h )

add_library( pcl_progs ${utils_srcs} ${utils_hdrs} )
target_link_libraries( pcl_progs ${PCL_LIBRARIES} )
set_target_properties( pcl_progs PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_executable( testBoundingBox tests/testBoundingBox.cpp )
target_link_libraries( testBoundingBox pcl_progs ${PCL_LIBRARIES} )
set_target_properties( testBoundingBox PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( testMesh2PCD tests/testMesh2PCD.cpp )
target_link_libraries( testMesh2PCD pcl_progs ${PCL_LIBRARIES} )
set_target_properties( testMesh2PCD PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

