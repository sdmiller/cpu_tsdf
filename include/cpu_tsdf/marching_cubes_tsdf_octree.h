/*
 * Software License Agreement
 *
 * Copyright (c) 2012-, Open Perception, Inc.
 * Author: Stephen Miller
 *  
 * All rights reserved.
 *
 */
#pragma once

#include <cpu_tsdf/tsdf_volume_octree.h>

#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/impl/marching_cubes.hpp>

namespace cpu_tsdf
{
  class MarchingCubesTSDFOctree : public pcl::MarchingCubes<pcl::PointXYZ>
  {
    using pcl::MarchingCubes<pcl::PointXYZ>::grid_;

  public:
    MarchingCubesTSDFOctree():
      color_by_confidence_ (false), 
      color_by_rgb_ (false), 
      w_min_ (2.5),
      pcl::MarchingCubes<pcl::PointXYZ> ()
    {}

    void
    setInputTSDF (cpu_tsdf::TSDFVolumeOctree::ConstPtr tsdf_volume);

    bool
    getValidNeighborList1D (std::vector<float> &leaf, 
                            Eigen::Vector3i &index3d);

    inline void
    setColorByConfidence (bool color_by_confidence)
    { color_by_confidence_ = color_by_confidence;}
    
    inline void
    setColorByRGB (bool color_by_rgb)
    { color_by_rgb_ = color_by_rgb;}

    inline void
    setMinWeight (float w_min)
    { w_min_ = w_min;}

  protected:
    void
    voxelizeData ();

    float
    getGridValue (Eigen::Vector3i pos);

    void
    performReconstruction (pcl::PolygonMesh &output);

    void
    reconstructVoxel (const OctreeNode *voxel, 
        pcl::PointCloud<pcl::PointXYZ> &output, 
        pcl::PointCloud<pcl::PointXYZRGB> *output_colored=NULL);

    cpu_tsdf::TSDFVolumeOctree::ConstPtr tsdf_volume_;
    bool color_by_confidence_;
    bool color_by_rgb_;
    float w_min_;
  };
}
