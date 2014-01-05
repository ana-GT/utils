/**
 * @file testVoxelRep
 */
#include "src/voxelRepresentation.h"

int main( int argc, char*argv[] ) {

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


  bool *grid;
  double leafSize = 0.01;
  int sizeX, sizeY, sizeZ;

  getGraph( input, grid,
	    leafSize,
	    sizeX, sizeY, sizeZ );
	    
  
  return 0;

}
