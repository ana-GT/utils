/**
 * @file getBoundingBox.cpp
 */
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigenvalues>
#include <pcl/visualization/pcl_visualizer.h>

#include "getBoundingBox.h"

/**
 * @function getBoundingBox
 * @brief Shows a visualizer with the bounding box and prints out its dimensions
 */
template<typename PointType>
void getBoundingBox( const boost::shared_ptr< pcl::PointCloud<PointType> > &_input  ) { 
 
  // Compute principal direction
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid( *_input, centroid );

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized( *_input, 
					  centroid, 
					  covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver( covariance, 
							      Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigenSolver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross( eigDx.col(1));
  
  // Move the points to that reference frame
  Eigen::Matrix4f p2w( Eigen::Matrix4f::Identity() );

  // [Rt, -Rt*T]
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.0*( p2w.block<3,3>(0,0) )*(centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> input_p;
  pcl::transformPointCloud( *_input, input_p, p2w );
  
  //-- Get max and min
  pcl::PointXYZ minP, maxP;
  pcl::getMinMax3D( input_p, minP, maxP );
  const Eigen::Vector3f meanDiag( 0.5*(minP.x + maxP.x), 
				  0.5*(minP.y + maxP.y), 
				  0.5*(minP.z + maxP.z));
  
  //-- Final transform
  const Eigen::Quaternionf qfinal( eigDx );
  const Eigen::Vector3f tfFinal = eigDx*meanDiag + centroid.head<3>();

  //-- Box Dimensions
  double bx = maxP.x - minP.x;
  double by = maxP.y - minP.y;
  double bz = maxP.z - minP.z;
  //-- Print info
  std::cout << " * Centroid: "<< centroid.head(3).transpose() << std::endl;
  std::cout << " * Dimensions: "<<bx<< ", "<<by<<", "<<bz<< std::endl; 

  //-- Show visualizer

  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud( _input );
  viewer.addCube( tfFinal, qfinal, bx, by, bz );
  viewer.addCoordinateSystem(1.0);
  viewer.spin();

  return;
}
