/*
 * Copyright (c) 2013-, Stephen Miller
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, 
 * with or without modification, are permitted provided 
 * that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the 
 * above copyright notice, this list of conditions 
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the 
 * above copyright notice, this list of conditions and 
 * the following disclaimer in the documentation and/or 
 * other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the 
 * names of its contributors may be used to endorse or
 * promote products derived from this software without 
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS 
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF 
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
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
