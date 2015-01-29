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

#include <pcl/common/transforms.h>
#include <pcl/filters/frustum_culling.h>
#include <assert.h>
#include <pcl/console/time.h>

template <typename PointT, typename NormalT> bool
cpu_tsdf::TSDFVolumeOctree::integrateCloud (
  const pcl::PointCloud<PointT> &cloud, 
  const pcl::PointCloud<NormalT> &normals,
  const Eigen::Affine3d &trans)
{
  Eigen::Affine3f trans_inv = trans.inverse ().cast<float> ();

  // First, sample a few points and force their containing voxels to split
  int px_step = 1;
  int nsplit = 0;
  for (size_t u = 0; u < cloud.width; u += px_step)
  {
    for (size_t v = 0; v < cloud.height; v += px_step)
    {
      const PointT &pt_surface_orig = cloud (u, v);
      if (pcl_isnan (pt_surface_orig.z))
        continue;
      // Look at surroundings
      int nstep = 0;
      Eigen::Vector3f ray = pt_surface_orig.getVector3fMap ().normalized ();
      for (int perm = 0; perm < num_random_splits_; perm++)
      {
        // Get containing voxels
        PointT pt_trans; 
        float scale = (float)rand () / (float)RAND_MAX * 0.03;
        Eigen::Vector3f noise = Eigen::Vector3f::Random ().normalized () * scale;;
        if (perm == 0) noise *= 0;
        pt_trans.getVector3fMap () = trans.cast<float> () * (pt_surface_orig.getVector3fMap ()+ noise);
        OctreeNode* voxel = octree_->getContainingVoxel (pt_trans.x, pt_trans.y, pt_trans.z);
        if (voxel != NULL)
        {
          while (voxel->getMinSize () > xsize_ / xres_)
          {
            nsplit++;
            voxel->split ();
            voxel = voxel->getContainingVoxel (pt_trans.x, pt_trans.y, pt_trans.z);
            
          }
        }
      }
    }
  }
  
  // Do Frustum Culling to get rid of unseen voxels
  std::vector<cpu_tsdf::OctreeNode::Ptr> voxels_culled;
  getFrustumCulledVoxels(trans, voxels_culled);
#pragma omp parallel for
  for (size_t i = 0; i < voxels_culled.size (); i++)
  {
    updateVoxel (voxels_culled[i], cloud, normals, trans_inv);
  }
  // Cloud is no longer empty
  is_empty_ = false;
  return (true);
}

// Utility function for the log normal distribution
inline float
logNormal (float x, float mean, float var)
{
  return (-std::pow (x - mean, 2) / (2*var));
}

// Returns 0 iff no measurement taken, 1 if a valid measurement was found, -1 if observed to be empty
template <typename PointT, typename NormalT> int
cpu_tsdf::TSDFVolumeOctree::updateVoxel (
    const cpu_tsdf::OctreeNode::Ptr &voxel, 
    const pcl::PointCloud<PointT> &cloud, 
    const pcl::PointCloud<NormalT> &normals, 
    const Eigen::Affine3f &trans_inv)
{
  // assert (!voxel->hasChildren ());

  if (voxel->hasChildren ())
  {
    std::vector<OctreeNode::Ptr>& children = voxel->getChildren ();
    std::vector<bool> is_empty (children.size ());
//#pragma omp parallel for
    for (size_t i = 0; i < children.size (); i++)
    {
      is_empty[i] = updateVoxel (children[i], cloud, normals, trans_inv) < 0;
    }
    bool all_are_empty = true;
    for (size_t i = 0; i < is_empty.size (); i++)
      all_are_empty &= is_empty[i];
    if (all_are_empty)
    {
      children.clear (); // Remove empty voxels
    }
    else
    {
      return (1); // Say I am not empty
    }
  }
  pcl::PointXYZ v_g_orig;
  voxel->getCenter (v_g_orig.x, v_g_orig.y, v_g_orig.z);
  pcl::PointXYZ v_g = pcl::transformPoint (v_g_orig, trans_inv);
  if (v_g.z < min_sensor_dist_ || v_g.z > max_sensor_dist_)
    return (0); // No observation
  int u, v;
  if (!reprojectPoint (v_g, u, v))
    return (0); // No observation
  const PointT &pt = cloud (u,v);
  if (pcl_isnan (pt.z))
    return (0); // No observation
   // if (pt.z >= max_sensor_dist_) // We want to let empty points be empty, even at noisy readings
   //   return (0);
  float d;
  float w;
  voxel->getData (d, w);
  float d_new = (pt.z - v_g.z);
  // Check if we can split
  if (fabs (d_new) < 3*voxel->getMaxSize ()/4.)// || (fabs (d_new) < max_dist_))  
  {
    float xsize, ysize, zsize;
    voxel->getSize (xsize, ysize, zsize);
    if (xsize > xsize_ / xres_ && ysize > ysize_ / yres_ && zsize > zsize_ / zres_)
    {
      std::vector<OctreeNode::Ptr>& children = voxel->split ();
      std::vector<bool> is_empty (children.size ());
//#pragma omp parallel for
      for (size_t i = 0; i < children.size (); i++)
      {
        // TODO Make the weight and distance match the parent's?
        // children[i]->setData (d, w);
        is_empty[i] = updateVoxel (children[i], cloud, normals, trans_inv) < 0;
      }
      bool all_are_empty = true;
      for (size_t i = 0; i < is_empty.size (); i++)
        all_are_empty &= is_empty[i];
      if (all_are_empty)
      {
        children.clear (); // Remove empty voxel
      }
      else
      {
        return (1); // Say I am not empty
      }
    }
  }
  if (d_new > max_dist_pos_)
  {
    d_new = max_dist_pos_;
  }
  else if (d_new < -max_dist_neg_)
  {
    return (0); // No observation, effectively
  }
  // Normalize s.t. -1 = unobserved
  d_new /= max_dist_neg_;

  float w_new = 1;
  if (weight_by_depth_)
    w_new *= (1 - std::min(pt.z / 10., 1.)); // Scales s.t. at 10m it's worthless
  if (weight_by_variance_ && voxel->nsample_ > 5)
    w_new *= std::exp (logNormal (d_new, voxel->d_, voxel->getVariance ()));
  if (integrate_color_)
    voxel->addObservation (d_new, w_new, max_weight_, pt.r, pt.g, pt.b);
  else
    voxel->addObservation (d_new, w_new, max_weight_);
  if (voxel->d_ < -0.99)
    return (0);
  else if (voxel->d_ < 0.99* max_dist_pos_ / max_dist_neg_) //if I am not empty
    return (1); // Say so
  else
    return (-1); // Otherwise say I'm empty
  
  
  
}

