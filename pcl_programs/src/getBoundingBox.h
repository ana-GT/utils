/**
 * @file getBoundingBox.h
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/shared_ptr.hpp>



template<typename PointType>
void getBoundingBox( const boost::shared_ptr< pcl::PointCloud<PointType> > &_input );

#include "getBoundingBox.hpp"
