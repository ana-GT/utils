/**
 * @file mesh2pcd.h
 */
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <string>

void ply2pcd( std::string _inputPLY,
	      std::string _outputPCD,
	      double _resolution = 100,
	      int _tesselated_sphere_level = 2,
	      double _leaf_size = 0.01f );

void obj2pcd( std::string _inputOBJ,
	      std::string _outputPCD,
	      double _resolution = 100,
	      int _tesselated_sphere_level = 2,
	      double _leaf_size = 0.01f );

void processPolydata( vtkSmartPointer<vtkPolyData> _polydata,
		      std::string _pcdFile,
		      double _resolution = 100,
		      int _tesselated_sphere_level = 2,
		      double _leaf_size = 0.01f );
