/**
 * @file testMesh2PCD.cpp
 */
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "src/mesh2pcd.h"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <ctime>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


void printHelp (int, char **argv);

int default_tesselated_sphere_level = 2;
int default_resolution = 100;
float default_leaf_size = 0.01f;

/**
 * @function main 
 */
int main (int argc, char **argv)
{
  clock_t startT;
  clock_t endT;
  double dt;

  print_info ("** Convert a CAD model to a point cloud using ray tracing operations. For more information, use: %s -h\n",
              argv[0]);
  
  if (argc < 3) {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  int tesselated_sphere_level = default_tesselated_sphere_level;
  parse_argument (argc, argv, "-level", tesselated_sphere_level);
  int resolution = default_resolution;
  parse_argument (argc, argv, "-resolution", resolution);
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf_size", leaf_size);

  // Parse the command line arguments for .ply and PCD files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
    {
      print_error ("Need a single output PCD file to continue.\n");
      return (-1);
    }
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (ply_file_indices.size () != 1 && obj_file_indices.size () != 1)
  {
    print_error ("Need a single input PLY/OBJ file to continue.\n");
    return (-1);
  }
  
  
  // Call mesh2pcd
  startT = clock();
  if (ply_file_indices.size () == 1) { 
    ply2pcd( argv[ply_file_indices[0]], argv[pcd_file_indices[0]] ); 
  } else if( obj_file_indices.size () == 1) {
    obj2pcd( argv[obj_file_indices[0]], argv[pcd_file_indices[0]] ); 
  }
  endT = clock();

  dt = (double) ( endT - startT ) / (double) CLOCKS_PER_SEC;
  printf( "[testMesh2PCD] Elapsed time: %f  seconds \n", dt );
}

/**
 * @function printHelp
 */
void printHelp (int, char **argv ) {

  print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -level X      = tesselated sphere level (default: ");
  print_value ("%d", default_tesselated_sphere_level);
  print_info (")\n");
  print_info ("                     -resolution X = the sphere resolution in angle increments (default: ");
  print_value ("%d", default_resolution);
  print_info (" deg)\n");
  print_info (
              "                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
  print_value ("%f", default_leaf_size);
  print_info (" m)\n");
}
