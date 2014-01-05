#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

int main( int argc, char* argv[] ) {

  if( argc != 3 ) {
    std::cout << "Usage: " << argv[0] << "input.pcd output.pcd" << std::endl;
    } 

  // Load
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );

    if( pcl::io::loadPCDFile( argv[1], *input ) == -1 ) {
        std::cout << " Error reading pointcloud. Exiting!"<< std::endl;
        return -1;
      }
  // Apply transform

    //-- Rotation
    double angle = 0.7;
    Eigen::Vector3d axis(1,0,0);
    Eigen::AngleAxis<double> Aa( angle, axis );
    //-- Translation
    Eigen::Vector3d trans( 0.2, 0.5, 1.2 );

    Eigen::Matrix4d Tf = Eigen::Matrix4d::Identity();
    Tf.block<3,3>(0,0) = Aa.matrix();
    Tf.block<3,1>(0,3) = trans;
    transformPointCloud( *input, *output, Tf ); 

    pcl::io::savePCDFileASCII ( argv[2], *output );
}
