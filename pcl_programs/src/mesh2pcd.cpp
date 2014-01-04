/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/apps/render_views_tesselated_sphere.h>

#include "mesh2pcd.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/**
 * @function obj2pcd
 */
void obj2pcd( std::string _inputOBJ,
	      std::string _outputPCD,
	      double _resolution,
	      int _tesselated_sphere_level,
	      double _leaf_size ) {

  vtkSmartPointer<vtkPolyData> polydata1;

  vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
  readerQuery->SetFileName ( _inputOBJ.c_str() );
  polydata1 = readerQuery->GetOutput ();
  polydata1->Update ();
  
  processPolydata( polydata1, _outputPCD );

}

/**
 * @function ply2pcd
 */
void ply2pcd( std::string _inputPLY,
	      std::string _outputPCD,
	      double _resolution,
	      int _tesselated_sphere_level,
	      double _leaf_size ) {

  vtkSmartPointer<vtkPolyData> polydata1;
  
  vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New ();

  readerQuery->SetFileName ( _inputPLY.c_str() );
  polydata1 = readerQuery->GetOutput ();
  polydata1->Update ();

  processPolydata( polydata1, _outputPCD );

}

/**
 * @functin processPolyData
 */
void processPolydata( vtkSmartPointer<vtkPolyData> _polydata,
		      std::string _pcdFile,
		      double _resolution,
		      int _tesselated_sphere_level,
		      double _leaf_size ) {

  bool INTER_VIS = false;
  bool VIS = true; 
    
  // Get views
  std::vector< PointCloud<PointXYZ>::Ptr > views_xyz; views_xyz.resize(0);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses; poses.resize(0);
  std::vector<float> enthropies; enthropies.resize(0);
  
  pcl::apps::RenderViewsTesselatedSphere render_views;
  render_views.addModelFromPolyData( _polydata );
  render_views.setResolution( _resolution );
  render_views.setComputeEntropies( false );
  render_views.setTesselationLevel( _tesselated_sphere_level );

  render_views.generateViews();

  render_views.getPoses( poses );
  render_views.getViews( views_xyz );
  //render_views.getEntropies( enthropies );

  //Take views and fuse them together
  std::vector<PointCloud<PointXYZ>::Ptr> aligned_clouds; 
  aligned_clouds.resize(0);
  
  for (size_t i = 0; i < views_xyz.size (); i++)
    {
      PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
      Eigen::Matrix4f pose_inverse;
      pose_inverse = poses[i].inverse ();
      transformPointCloud ( *views_xyz[i], *cloud, pose_inverse);
      aligned_clouds.push_back (cloud);
    }
  
  // ** INTER_VIS **
  if (INTER_VIS)
    {
      visualization::PCLVisualizer vis2 ("visualize");
      
      for (size_t i = 0; i < aligned_clouds.size (); i++)
	{
	  std::stringstream name;
	  name << "cloud_" << i;
	  vis2.addPointCloud (aligned_clouds[i], name.str ());
	  
	  vis2.spin (); 
	}
    }
  
  // Fuse clouds
  PointCloud<PointXYZ>::Ptr big_boy (new PointCloud<PointXYZ> ());
  big_boy->points.resize(0);
  for (size_t i = 0; i < aligned_clouds.size (); i++)
    *big_boy += *aligned_clouds[i];
  
  // ** VIS **
  if (VIS)
    {
      visualization::PCLVisualizer vis2 ("visualize");
      vis2.addPointCloud (big_boy);
      vis2.spin (); 
    }
  
  // Voxelgrid
  PointCloud<PointXYZ>::Ptr big_boyFiltered (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> grid_;
  grid_.setInputCloud (big_boy);
  grid_.setLeafSize ( _leaf_size, _leaf_size, _leaf_size);
  grid_.filter (*big_boyFiltered);
  
  if (VIS)
    {
      visualization::PCLVisualizer vis3 ("visualize");
      vis3.addPointCloud (big_boy);
      vis3.spin (); 
    }
  
  printf("Saving to PCD \n");
  savePCDFileASCII ( _pcdFile, *big_boyFiltered);

  printf(" Closing visualization windows \n");

  return;
}
