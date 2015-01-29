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


#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

void
cpu_tsdf::MarchingCubesTSDFOctree::setInputTSDF (cpu_tsdf::TSDFVolumeOctree::ConstPtr tsdf_volume)
{
  tsdf_volume_ = tsdf_volume;
  // Set the grid resolution so it mimics the tsdf's
  int res_x, res_y, res_z;
  tsdf_volume_->getResolution (res_x, res_y, res_z);
  setGridResolution (res_x, res_y, res_z);
  float size_x, size_y, size_z;
  tsdf_volume_->getGridSize (size_x, size_y, size_z);
  // Set the "input cloud" to the 8 corners of the tsdf, to trick our parent
  // into voxelizing it the same
  pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int x_i = 0; x_i <= res_x; x_i += res_x)
  {
    for (int y_i = 0; y_i <= res_y; y_i += res_y)
    {
      for (int z_i = 0; z_i <= res_z; z_i += res_z)
      {
        pcl::PointXYZ center = tsdf_volume_->getVoxelCenter (x_i, y_i, z_i);
        // Go from center to corner
        center.x += (x_i == 0 ? -1 : 1) * 0.5 * size_x / res_x;
        center.y += (y_i == 0 ? -1 : 1) * 0.5 * size_y / res_y;
        center.z += (z_i == 0 ? -1 : 1) * 0.5 * size_z / res_z;
        corner_cloud->points.push_back (center);
      }
    }
  }
  setInputCloud (corner_cloud);
  // No extending
  setPercentageExtendGrid (0);
  float iso_x = 0;
  float iso_y = 0;
  float iso_z = 0;
  PCL_INFO ("iso_x = %f, iso_y = %f, iso_z = %f\n", iso_x, iso_y, iso_z);
  setIsoLevel (std::min (iso_x, std::min (iso_y, iso_z)));
}

void
cpu_tsdf::MarchingCubesTSDFOctree::voxelizeData ()
{
  // Do nothing, extending getGridValue instead to save memory
}

float
cpu_tsdf::MarchingCubesTSDFOctree::getGridValue (Eigen::Vector3i pos)
{
  pcl::PointXYZ ctr = tsdf_volume_->getVoxelCenter (pos[0], pos[1], pos[2]);
  const cpu_tsdf::OctreeNode* voxel = tsdf_volume_->octree_->getContainingVoxel (ctr.x, ctr.y, ctr.z);
  float d;
  float w;
  voxel->getData (d, w);
  //float size_x, size_y, size_z;
  //voxel->getSize (size_x, size_y, size_z);
  if (w < w_min_ || fabs(d) >= 1)
    return (std::numeric_limits<float>::quiet_NaN ());
  float max_dist_pos, max_dist_neg;
  tsdf_volume_->getDepthTruncationLimits (max_dist_pos, max_dist_neg);
  return (d * max_dist_neg); //was fabs
}
    
void
cpu_tsdf::MarchingCubesTSDFOctree::performReconstruction (pcl::PolygonMesh &output)
{
  getBoundingBox ();
  voxelizeData ();
  // Run the actual marching cubes algorithm, store it into a point cloud,
  // and copy the point cloud + connectivity into output
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Recursive reconstruction
  OctreeNode::Ptr root = tsdf_volume_->octree_->getRoot ();
  if (color_by_confidence_ || color_by_rgb_)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
    reconstructVoxel (root.get (), cloud, &cloud_colored);
    pcl::transformPointCloud (cloud_colored, cloud_colored, tsdf_volume_->getGlobalTransform ());
    pcl::toPCLPointCloud2 (cloud_colored, output.cloud);
  }
  else
  {
    reconstructVoxel (root.get (), cloud);
    pcl::transformPointCloud (cloud, cloud, tsdf_volume_->getGlobalTransform ());
    pcl::toPCLPointCloud2 (cloud, output.cloud);
  }

  output.polygons.resize (cloud.size () / 3);
  for (size_t i = 0; i < output.polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.resize (3);
    for (int j = 0; j < 3; ++j)
    {
      v.vertices[j] = static_cast<int> (i) * 3 + j;
    }
    output.polygons[i] = v;
  }
}
    
bool
cpu_tsdf::MarchingCubesTSDFOctree::getValidNeighborList1D (std::vector<float> &leaf, 
                                                     Eigen::Vector3i &index3d)
{
  leaf = std::vector<float> (8, 0.0f);

  leaf[0] = getGridValue (index3d);
  if (pcl_isnan (leaf[0]))
    return (false);
  leaf[1] = getGridValue (index3d + Eigen::Vector3i (1, 0, 0));
  if (pcl_isnan (leaf[1]))
    return (false);
  leaf[2] = getGridValue (index3d + Eigen::Vector3i (1, 0, 1));
  if (pcl_isnan (leaf[2]))
    return (false);
  leaf[3] = getGridValue (index3d + Eigen::Vector3i (0, 0, 1));
  if (pcl_isnan (leaf[3]))
    return (false);
  leaf[4] = getGridValue (index3d + Eigen::Vector3i (0, 1, 0));
  if (pcl_isnan (leaf[4]))
    return (false);
  leaf[5] = getGridValue (index3d + Eigen::Vector3i (1, 1, 0));
  if (pcl_isnan (leaf[5]))
    return (false);
  leaf[6] = getGridValue (index3d + Eigen::Vector3i (1, 1, 1));
  if (pcl_isnan (leaf[6]))
    return (false);
  leaf[7] = getGridValue (index3d + Eigen::Vector3i (0, 1, 1));
  if (pcl_isnan (leaf[7]))
    return (false);
  return (true);

}

void
cpu_tsdf::MarchingCubesTSDFOctree::reconstructVoxel (const OctreeNode* voxel, pcl::PointCloud<pcl::PointXYZ> &output, pcl::PointCloud<pcl::PointXYZRGB>* output_colored)
{
  if (voxel->hasChildren ())
  {
    const std::vector <OctreeNode::Ptr> &children = voxel->getChildren ();
    for (size_t i = 0; i < children.size (); i++)
      reconstructVoxel (children[i].get (), output, output_colored);
  }
  else
  {
    float d;
    float w;
    voxel->getData (d, w);
    if (w >= w_min_ && fabs (d) < 1)
    {
      float x, y, z;
      voxel->getCenter (x, y, z);
      Eigen::Vector3i idx;
      tsdf_volume_->getVoxelIndex (x, y, z, idx (0), idx (1), idx (2));
      if (idx (0) <= 0 || idx (0) >= res_x_-1 ||
          idx (1) <= 0 || idx (1) >= res_y_-1 ||
          idx (2) <= 0 || idx (2) >= res_z_-1)
        return;
      // Reconstruct
      std::vector<float> leaf_node;
      if (!getValidNeighborList1D (leaf_node, idx))
        return;
      createSurface (leaf_node, idx, output);
      if (output_colored != NULL)
      {
        for (size_t i = output_colored->size (); i < output.size (); ++i)
        {
          pcl::PointXYZRGB pt_rgb;
          pt_rgb.getVector3fMap () = output.at (i).getVector3fMap ();
          // Color by variance instead
          uint8_t r, g, b;
          if (color_by_confidence_)
          {
            //float std_dev = std::sqrt (voxel->getVariance ());
            float std_dev = (100. - voxel->w_)/100.;
            pt_rgb.r = std::max (0., std::min ((1-std_dev)*255., 255.));
            pt_rgb.g = 0;
            pt_rgb.b = std::max (0., std::min ((std_dev)*255., 255.));
          }
          // Color by rgb
          else if (color_by_rgb_ && voxel->getRGB (r, g, b))
          {
            pt_rgb.r = r;
            pt_rgb.g = g;
            pt_rgb.b = b;
          }
          output_colored->push_back (pt_rgb);
        }
      }
    }
  }
}

