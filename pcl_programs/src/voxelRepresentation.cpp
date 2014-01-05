/**
 * @file voxelRepresentation.cpp
 */
#include "voxelRepresentation.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

/**
 * @function getGraph
 */
void getGraph( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	       bool grid[],
	       const double &_leafSize,
	       int &_sizeX, 
	       int &_sizeY, 
	       int &_sizeZ );

/**
 * @function getGraph
 */
void getGraph( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	       bool grid[],
	       const double &_leafSize,
	       int &_sizeX, 
	       int &_sizeY, 
	       int &_sizeZ ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered( new pcl::PointCloud<pcl::PointXYZ>() );

  
  pcl::VoxelGrid<pcl::PointXYZ> voxels;
  voxels.setInputCloud( _cloud );

  voxels.setLeafSize( _leafSize, _leafSize, _leafSize );
  voxels.filter( *cloudFiltered );

  Eigen::Vector3i numDiv = voxels.getNrDivisions();
  _sizeX = numDiv(0);
  _sizeY = numDiv(1);
  _sizeZ = numDiv(2);

  int numCells = _sizeX*_sizeY*_sizeZ;

  grid = new bool[numCells ];
  for( int i = 0; i < cloudFiltered->points.size(); ++i ) {
    grid[i] = false;
  }

  
  Eigen::Vector3i minBC = voxels.getMinBoxCoordinates();
  Eigen::Vector3i maxBC = voxels.getMaxBoxCoordinates();
  
  Eigen::Vector3i coord;
  int count = 0;
  int negCount = 0;
  for( int i = 0; i < cloudFiltered->points.size(); ++i ) {
    coord = voxels.getGridCoordinates( cloudFiltered->points[i].x,
					cloudFiltered->points[i].y,
					cloudFiltered->points[i].z );
    if( coord(0) < 0 || coord(1) < 0 || coord(2) < 0 ) {
      negCount++;
    }
    int x, y, z;
    x = coord(0) - minBC(0);
    y = coord(1) - minBC(1);
    z = coord(2) - minBC(2);
    grid[ x*(_sizeX*_sizeY) + y*_sizeY + z] = true;
    count++;
  } 
  std::cout << "Occupied voxels: "<< count << std::endl;
  std::cout << "Neg counts: "<< negCount << std::endl;


  std::cout << "Num div: "<< numDiv.transpose() << std::endl;
  std::cout << " Min Box coordinates: "<< minBC.transpose() << std::endl;
  std::cout << " Max Box coordinates: "<< maxBC.transpose() << std::endl;

  pcl::visualization::PCLVisualizer viewer;
  
  for( int i = 0; i < cloudFiltered->points.size(); ++i ) {
    
    char name[30];
    sprintf(name, "cube%d",i);
    viewer.addCube( Eigen::Vector3f(cloudFiltered->points[i].x,
				    cloudFiltered->points[i].y,
				    cloudFiltered->points[i].z),
		    Eigen::Quaternionf::Identity(),
		    0.1*_leafSize, 0.1*_leafSize, 0.1*_leafSize,
		    name );
  }
  
  /*
  viewer.addPointCloud( cloudFiltered, "SC1" );
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "SC1");
  */
  viewer.addCoordinateSystem(1.0);
  viewer.spin();
  
  return;
}
