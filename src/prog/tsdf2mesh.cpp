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

