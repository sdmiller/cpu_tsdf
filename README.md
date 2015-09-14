This package provides pcl-compatible tools for building a smooth volumetric surface from registered point clouds, following a similar 
approach to that of Curless and Levoy, as made popular by KinectFusion. This runs on a CPU, using an octree to 
efficiently compress free space and scale to larger environments than can fit on a GPU. Also comes with 
a Marching Cube implementation, for extracting the isosurface.

Note: This tool is meant primarily for exploring the basics of volumetric reconstruction / mocking up ideas in a research setting. Everyone says their code is "unoptimized" as a sort of Get Out Of Jail Free Card in academia, but here it's true by design: I represent the volume as a collection of pointers to (abstract) Voxel objects, and make all calls recursively...including I/O calls. That has the benefit of being very simple to modify / extend, while seriously hurting performance. As such, I'd suggest using cpu_tsdf only as a research tool for figuring out what parameters/architecture you'd like. Once you've settled on it, you'll probably want to homebrew your own solution before deploying it in the wild.

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

Handy Tools
=====

To help get the ball rolling, I have one major executable, **./integrate**.

Usage: integrate --in path/to/pcd_directory --out path/to/desired/output [many optional parameters, use --help to view]

As the long list of parameters should clue you in to, this is a fairly versatile program. Its job is to read in a directory of .pcd files and .txt files, where foo.txt is a 4 row ASCII matrix representing the pose of foo.pcd in some arbitrary reference frame (typically such that the first PCD has an identity pose.) The contents of foo.txt may look like:

```
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1 //last row optional
```

By convention, this is the transform which can be applied to the cloud to bring it into the world coordinate system. Note that this is identical to the Eigen::Matrix4f which pcl::IterativeClosestPoint, and other pcl registration techniques, return when aligning cloud[i] to some world cloud. Note also that if you were to use ICP to align clouds pairwise, transform[i->world] = transform[i->i-1]\*transform[i-1->i-2]\*...\*transform[0->world]. Hint hint. If you use the opposite convention (the pose is the world in the current camera frame) you can use the --invert option.

This will create a file at path/to/desired/output/mesh.ply.

To help adapt parameters for your sensor, I added the **./get_intrinsics** tool. Provide it an organized point cloud from any sensor (i.e. has cloud.width, cloud.height, and either NaN or a point at all width*height locations) and it will output the best guess at focal length and center pixel.

Note on the codebase
=====

You'll notice some extraneous-seeming layouts to the codebase: eigen\_extensions vs cpu\_tsdf, TSDFInterface as a parent of TSDFVolumeOctree. At the moment this probably seems unnecessary; it's a biproduct of the fact that other methods (GPU-based as used by KinectFyusion, CPU with no octree as used by my masochistic past-self) have also existed in unstable forms, in larger projects. I leave this layer of 
abstraction here on the off chance that I port the others to the project (modulo public interest), but feel 
free to ignore it for now and judge me accordingly.
