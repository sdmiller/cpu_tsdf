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


#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>

#include <string>
#include <vector>

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    PCL_INFO ("This is a utility program meant to render a mesh from a TSDF Volume, which can be saved to disk via TSDFVolumeOctree::save(const std::string &filename).\n");
    PCL_INFO ("Usage: %s foo.vol foo.ply", argv[0]);
    return (1);
  }
  std::string volume_file = argv[1];
  std::string mesh_file = argv[2];
  PCL_INFO ("Converting %s -> %s\n", volume_file.c_str (), mesh_file.c_str ());
  cpu_tsdf::TSDFInterface::Ptr tsdf = cpu_tsdf::TSDFInterface::instantiateFromFile (volume_file);
  PCL_INFO ("Loaded! Running marching cubes\n");
  cpu_tsdf::MarchingCubesTSDFOctree mc;
  mc.setInputTSDF (boost::dynamic_pointer_cast<cpu_tsdf::TSDFVolumeOctree> (tsdf));
  mc.setColorByConfidence (false);
  mc.setColorByRGB (false);
  pcl::PolygonMesh mesh;
  mc.reconstruct (mesh);
  pcl::io::savePLYFileBinary (mesh_file, mesh);

}

