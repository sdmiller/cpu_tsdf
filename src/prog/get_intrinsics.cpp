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


#include <pcl/console/print.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <vector>

void
getIntrinsics (const pcl::PointCloud<pcl::PointXYZ> &cloud,
              float &fx,
              float &fy,
              float &cx,
              float &cy,
              float &reproj_error)
{
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero (2*cloud.size (), 4);
  Eigen::VectorXf b = Eigen::VectorXf::Zero (2*cloud.size ());
  size_t idx = 0;
  for (int x = 0 ; x < cloud.width; x++)
  {
    for (int y = 0; y < cloud.height; y++)
    {
      const pcl::PointXYZ &pt = cloud (x,y);
      if (pcl_isnan (pt.x) || pcl_isnan (pt.y) || pcl_isnan (pt.z) || (pt.x == 0) || (pt.y == 0))
        continue;
      A (idx, 0) = pt.z;
      A (idx, 2) = pt.x;
      b (idx) = pt.z * x;
      idx++;
      A (idx, 1) = pt.z;
      A (idx, 3) = pt.y;
      b (idx) = pt.z * y;
      idx++;
    }
  }
  // Solve A*x = b;
  Eigen::Vector4f X;
  X = (A.transpose () * A).inverse () * A.transpose () * b;
  cx = X (0);
  cy = X (1);
  fx = X (2);
  fy = X (3);
  reproj_error = (A*X - b).squaredNorm () / (fx*fx);
}

int
main (int argc, char** argv)
{
  if (argc != 2)
  {
    PCL_INFO ("Usage: %s input_organized_cloud.pcd\n", argv[0]);
    return (1);
  }
  std::string pcd_file = argv[1];
  pcl::PointCloud<pcl::PointXYZ> cloud;
  PCL_INFO ("Loading cloud %s\n", pcd_file.c_str ());
  pcl::io::loadPCDFile (pcd_file, cloud);
  float fx, fy, cx, cy, reproj_error;
  getIntrinsics (cloud, fx, fy, cx, cy, reproj_error);
  PCL_INFO ("Width: %d\n", cloud.width);
  PCL_INFO ("Height: %d\n", cloud.height);
  PCL_INFO ("fx: %f\n", fx);
  PCL_INFO ("fy: %f\n", fy);
  PCL_INFO ("cx: %f\n", cx);
  PCL_INFO ("cy: %f\n", cy);
  PCL_INFO ("Total reprojection error: %f\n", reproj_error);
  return (0);
}
