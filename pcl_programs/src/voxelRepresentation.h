/**
 * @file voxelRepresentation.h
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/**
 * @function getGraph
 */
void getGraph( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	       bool grid[],
	       const double &_leafSize,
	       int &_sizeX, 
	       int &_sizeY, 
	       int &_sizeZ );
