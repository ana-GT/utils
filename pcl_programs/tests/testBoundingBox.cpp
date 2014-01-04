/**
 * @file testBoundingBox.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "src/getBoundingBox.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  if( argc != 2 ) {
    std::cout << "Usage: "<< argv[0]<<" pointcloud.pcd"<< std::endl;
    return -1;
  }

  // Load pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  
  if( pcl::io::loadPCDFile( argv[1], *input ) == -1 ) {
    std::cout << "[!] Error reading input pointcloud. Returning!"<< std::endl;
    return -1;
  }
  
  // Get bounding box and display it in a viewer
  getBoundingBox<pcl::PointXYZ>( input );

  return 0;
}
