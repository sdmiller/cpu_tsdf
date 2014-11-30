This package provides pcl-compatible tools for building a smooth volumetric surface from registered point clouds, following a similar 
approach to that of Curless and Levoy, as made popular by KinectFusion. This runs on a CPU, using an octree to 
efficiently compress free space and scale to larger environments than can fit on a GPU. Also comes with 
a Marching Cube implementation, for extracting the isosurface.

Build Instructions
===================
```bash
mkdir build && cd build && cmake .. && make
```

Usage
=====

Let's say you have a list of PointClouds (in the sensor frame) and the camera poses. To merge them into a volume:

```cpp
 TSDFVolumeOctree::Ptr tsdf (new TSDFVolumeOctree);
 tsdf->setGridSize (10., 10., 10.); // 10m x 10m x 10m
 tsdf->setResolution (2048, 2048, 2048); // Smallest cell size = 10m / 2048 = about half a centimeter
 tsdf->setIntegrateColor (false); // Set to true if you want the TSDF to store color
 Eigen::Affine3d tsdf_center; // Optionally offset the center
 tsdf->setGlobalTransform (tsdf_center);
 tsdf->reset (); // Initialize it to be empty
 for (size_t i = 0; i < clouds.size (); i++)
 {
   tsdf->integrateCloud (clouds[i], normals[i], poses[i]); // Integrate the cloud
   // Note, the normals aren't being used in the default settings. Feel free to pass in an empty cloud
 }
 // Now what do you want to do with it? 
 float distance; pcl::PointXYZ query_point (1.0, 2.0, -1.0);
 tsdf->getFxn (query_point, distance); // distance is normalized by the truncation limit -- goes from -1 to 1
 pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView (pose_to_render_from); // Optionally can render it
 tsdf->save ("output.vol"); // Save it?  
 
 // Mesh with marching cubes
 MarchingCubesTSDFOctree mc;
 mc.setInputTSDF (tsdf);
 mc.setMinWeight (2); // Sets the minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
 mc.setColorByRGB (false); // If true, tries to use the RGB values of the TSDF for meshing -- required if you want a colored mesh
 pcl::PolygonMesh mesh;
 mc.reconstruct (mesh);
```

These are just the basics. You can add colors, store your own metadata in voxels (see octree.h), etc. Please ping me for bugs!

