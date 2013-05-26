#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>

cpu_tsdf::TSDFInterface::Ptr
cpu_tsdf::TSDFInterface::instantiateFromFile (const std::string &filename)
{
  // For now everything is an octree.
  TSDFInterface::Ptr tsdf (new TSDFVolumeOctree);
  tsdf->load (filename);
  return (tsdf);
}
