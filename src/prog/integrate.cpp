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


#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <vector>

int width_ = 640;
int height_ = 480;
float focal_length_x_ = 525.;
float focal_length_y_ = 525.;
float principal_point_x_ = 319.5;
float principal_point_y_ = 239.5;

pcl::PointCloud<pcl::PointNormal>::Ptr
meshToFaceCloud (const pcl::PolygonMesh &mesh)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2 (mesh.cloud, vertices);

  for (size_t i = 0; i < mesh.polygons.size (); ++i)
  {
    if (mesh.polygons[i].vertices.size () != 3)
    {
      PCL_ERROR ("Found a polygon of size %d\n", mesh.polygons[i].vertices.size ());
      continue;
    }
    Eigen::Vector3f v0 = vertices.at (mesh.polygons[i].vertices[0]).getVector3fMap ();
    Eigen::Vector3f v1 = vertices.at (mesh.polygons[i].vertices[1]).getVector3fMap ();
    Eigen::Vector3f v2 = vertices.at (mesh.polygons[i].vertices[2]).getVector3fMap ();
    float area = ((v1 - v0).cross (v2 - v0)).norm () / 2. * 1E4;
    Eigen::Vector3f normal = ((v1 - v0).cross (v2 - v0));
    normal.normalize ();
    pcl::PointNormal p_new;
    p_new.getVector3fMap () = (v0 + v1 + v2)/3.;
    p_new.normal_x = normal (0);
    p_new.normal_y = normal (1);
    p_new.normal_z = normal (2);
    cloud->points.push_back (p_new);
  }
  cloud->height = 1;
  cloud->width = cloud->size ();
  return (cloud);
}

void
flattenVertices (pcl::PolygonMesh &mesh, float min_dist = 0.0001)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (mesh.cloud, *vertices);
  pcl::search::KdTree<pcl::PointXYZ> vert_tree (true);
  vert_tree.setInputCloud (vertices);
  // Find duplicates
  std::vector<int> vertex_remap (vertices->size (), -1);
  int idx = 0;
  std::vector<int> neighbors;
  std::vector<float> dists;
  pcl::PointCloud<pcl::PointXYZ> vertices_new;
  for (size_t i = 0; i < vertices->size (); i++)
  {
    if (vertex_remap[i] >= 0)
      continue;
    vertex_remap[i] = idx;
    vert_tree.radiusSearch (i, min_dist, neighbors, dists);
    for (size_t j = 1; j < neighbors.size (); j++)
    {
      if (dists[j] < min_dist)
        vertex_remap[neighbors[j]] = idx;
    }
    vertices_new.push_back (vertices->at (i));
    idx++;
  }
  std::vector<size_t> faces_to_remove;
  size_t face_idx = 0;
  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    pcl::Vertices &v = mesh.polygons[i];
    for (size_t j = 0; j < v.vertices.size (); j++)
    {
      v.vertices[j] = vertex_remap[v.vertices[j]];
    }
    if (v.vertices[0] == v.vertices[1] || v.vertices[1] == v.vertices[2] || v.vertices[2] == v.vertices[0])
    {
      PCL_INFO ("Degenerate face: (%d, %d, %d)\n", v.vertices[0], v.vertices[1], v.vertices[2]);
    }
    else
    {
      mesh.polygons[face_idx++] = mesh.polygons[i];
    }
  }
  mesh.polygons.resize (face_idx);
  pcl::toPCLPointCloud2 (vertices_new, mesh.cloud);
}

void
cleanupMesh (pcl::PolygonMesh &mesh, float face_dist=0.02, int min_neighbors=5)
{
  // Remove faces which aren't within 2 marching cube widths from any others
  pcl::PointCloud<pcl::PointNormal>::Ptr faces = meshToFaceCloud (mesh);
  std::vector<size_t> faces_to_remove;
  pcl::search::KdTree<pcl::PointNormal>::Ptr face_tree (new pcl::search::KdTree<pcl::PointNormal>);
  face_tree->setInputCloud (faces);
  // Find small clusters and remove them
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointNormal> extractor;
  extractor.setInputCloud (faces);
  extractor.setSearchMethod (face_tree);
  extractor.setClusterTolerance (face_dist);
  extractor.setMaxClusterSize(min_neighbors);
  extractor.extract(clusters);
  PCL_INFO ("Found %d clusters\n", clusters.size ());
  // Aggregate indices
  std::vector<bool> keep_face (faces->size (), false);
  for(size_t i = 0; i < clusters.size(); i++)
  {
    for(size_t j = 0; j < clusters[i].indices.size(); j++)
    {
      faces_to_remove.push_back (clusters[i].indices[j]);
    }
  }
  std::sort (faces_to_remove.begin (), faces_to_remove.end ());
  // Remove the face
  for (ssize_t i = faces_to_remove.size () - 1; i >= 0; i--)
  {
    mesh.polygons.erase (mesh.polygons.begin () + faces_to_remove[i]);
  }
  // Remove all vertices with no face
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2 (mesh.cloud, vertices);
  std::vector<bool> has_face (vertices.size (), false);
  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    const pcl::Vertices& v = mesh.polygons[i];
    has_face[v.vertices[0]] = true;
    has_face[v.vertices[1]] = true;
    has_face[v.vertices[2]] = true;
  }
  pcl::PointCloud<pcl::PointXYZ> vertices_new;
  std::vector<size_t> get_new_idx (vertices.size ());
  size_t cur_idx = 0;
  for (size_t i = 0; i <vertices.size (); i++)
  {
    if (has_face[i])
    {
      vertices_new.push_back (vertices[i]);
      get_new_idx[i] = cur_idx++;
    }
  }
  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    pcl::Vertices &v = mesh.polygons[i];
    v.vertices[0] = get_new_idx[v.vertices[0]];
    v.vertices[1] = get_new_idx[v.vertices[1]];
    v.vertices[2] = get_new_idx[v.vertices[2]];
  }
  pcl::toPCLPointCloud2 (vertices_new, mesh.cloud);
}

bool
reprojectPoint (const pcl::PointXYZRGBA &pt, int &u, int &v)
{
  u = (pt.x * focal_length_x_ / pt.z) + principal_point_x_;
  v = (pt.y * focal_length_y_ / pt.z) + principal_point_y_;
  return (!pcl_isnan (pt.z) && pt.z > 0 && u >= 0 && u < width_ && v >= 0 && v < height_);
}

std::string getSharedPrefix (const std::vector<std::string> &files)
{
  // Compare the first and last (sorted) string
  const std::string &first = files.front ();
  const std::string &last = files.back ();
  // March til we break or hit a numeric character;
  int i;
  for (i = 0; i < first.length (); i++)
  {
    if (first[i] != last[i] || std::isdigit(first[i]))
    {
      break;
    }
  }
  if (i == 0)
  {
    return ("");
  }
  else
  {
    return (first.substr (0, i));
  }
}


int
main (int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("in", bpo::value<std::string> ()->required (), "Input dir")
    ("out", bpo::value<std::string> ()->required (), "Output dir")
    ("volume-size", bpo::value<float> (), "Volume size")
    ("cell-size", bpo::value<float> (), "Cell size")
    ("num-frames", bpo::value<size_t> (), "Partially integrate the sequence: only the first N clouds used")
    ("visualize", "Visualize")
    ("verbose", "Verbose")
    ("color", "Store color in addition to depth in the TSDF")
    ("flatten", "Flatten mesh vertices")
    ("cleanup", "Clean up mesh")
    ("invert", "Transforms are inverted (world -> camera)")
    ("world", "Clouds are given in the world frame")
    ("organized", "Clouds are already organized")
    ("width", bpo::value<int> (), "Image width")
    ("height", bpo::value<int> (), "Image height")
    ("zero-nans", "Nans are represented as (0,0,0)")
    ("num-random-splits", bpo::value<int> (), "Number of random points to sample around each surface reading. Leave empty unless you know what you're doing.")
    ("fx", bpo::value<float> (), "Focal length x")
    ("fy", bpo::value<float> (), "Focal length y")
    ("cx", bpo::value<float> (), "Center pixel x")
    ("cy", bpo::value<float> (), "Center pixel y")
    ("save-ascii", "Save ply file as ASCII rather than binary")
    ("cloud-units", bpo::value<float> (), "Units of the data, in meters")
    ("pose-units", bpo::value<float> (), "Units of the poses, in meters")
    ("max-sensor-dist", bpo::value<float> (), "Maximum distance data can be from the sensor")
    ("min-sensor-dist", bpo::value<float> (), "Minimum distance data can be from the sensor")
    ("trunc-dist-pos", bpo::value<float> (), "Positive truncation distance")
    ("trunc-dist-neg", bpo::value<float> (), "Negative truncation distance")
    ("min-weight", bpo::value<float> (), "Minimum weight to render")
    ("cloud-only", "Save aggregate cloud rather than actually running TSDF")
    ;
     
  bpo::variables_map opts;
  bpo::store(bpo::parse_command_line(argc, argv, opts_desc, bpo::command_line_style::unix_style ^ bpo::command_line_style::allow_short), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " --in [in_dir] --out [out_dir] [OPTS]" << endl;
    cout << "Integrates multiple clouds and returns a mesh. Assumes clouds are PCD files and poses are ascii (.txt) or binary float (.transform) files with the same prefix, specifying the pose of the camera in the world frame. Can customize many parameters, but if you don't know what they do, the defaults are strongly recommended." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return (1);
  }

  // Visualize?
  bool visualize = opts.count ("visualize");
  bool verbose = opts.count ("verbose");
  bool flatten = opts.count ("flatten");
  bool cleanup = opts.count ("cleanup");
  bool invert = opts.count ("invert");
  bool organized = opts.count ("organized");
  bool world_frame = opts.count ("world");
  bool zero_nans = opts.count ("zero-nans");
  bool save_ascii = opts.count ("save-ascii");
  float cloud_units = 1.;
  if (opts.count ("cloud-units"))
    cloud_units = opts["cloud-units"].as<float> ();
  float pose_units = 1.;
  if (opts.count ("pose-units"))
    pose_units = opts["pose-units"].as<float> ();
  int num_random_splits = 1;
  if (opts.count ("num-random-splits"))
    num_random_splits = opts["num-random-splits"].as<int> ();
  float max_sensor_dist = 3.0;
  if (opts.count ("max-sensor-dist"))
    max_sensor_dist = opts["max-sensor-dist"].as<float> ();
  float min_sensor_dist = 0;
  if (opts.count ("min-sensor-dist"))
    min_sensor_dist = opts["min-sensor-dist"].as<float> ();
  float min_weight = 0;
  if (opts.count ("min-weight"))
    min_weight = opts["min-weight"].as<float> ();
  float trunc_dist_pos = 0.03;
  if (opts.count ("trunc-dist-pos"))
    trunc_dist_pos = opts["trunc-dist-pos"].as<float> ();
  float trunc_dist_neg = 0.03;
  if (opts.count ("trunc-dist-neg"))
    trunc_dist_neg = opts["trunc-dist-neg"].as<float> ();
  bool binary_poses = false;
  if (opts.count ("width"))
    width_ = opts["width"].as<int> ();
  if (opts.count ("height"))
    height_ = opts["height"].as<int> ();
  focal_length_x_ = 525. * width_ / 640.;
  focal_length_y_ = 525. * height_ / 480.;
  principal_point_x_ = static_cast<float> (width_)/2. - 0.5;
  principal_point_y_ = static_cast<float> (height_)/2. - 0.5;

  if (opts.count ("fx"))
    focal_length_x_ = opts["fx"].as<float> ();
  if (opts.count ("fy"))
    focal_length_y_ = opts["fy"].as<float> ();
  if (opts.count ("cx"))
    principal_point_x_ = opts["cx"].as<float> ();
  if (opts.count ("cy"))
    principal_point_y_ = opts["cy"].as<float> ();
  
  bool cloud_only = opts.count ("cloud-only");
  bool integrate_color = opts.count("color");

  pcl::console::TicToc tt;
  tt.tic ();
  // Scrape files
  std::vector<std::string> pcd_files;
  std::vector<std::string> pose_files;
  std::vector<std::string> pose_files_unordered;
  bool found_pose_file = false;
  std::string pose_extension = "";
  std::string dir = opts["in"].as<std::string> ();
  std::string out_dir = opts["out"].as<std::string> ();
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    std::string extension = boost::filesystem::extension (itr->path ());
    std::string pathname = itr->path ().string ();
    // Look for PCD files
    if (extension == ".PCD" || extension == ".pcd")
    {
      pcd_files.push_back (pathname);
    }
    else if (extension == ".TRANSFORM" || extension == ".transform")
    {
      if (found_pose_file && extension != pose_extension)
      {
        PCL_ERROR ("Files with extension %s and %s were found in this folder! Please choose a consistent extension.\n", extension.c_str (), pose_extension.c_str ());
        return (1);
      }
      else if (!found_pose_file)
      {
        found_pose_file = true;
        binary_poses = true;
        pose_extension = extension;
      }
      pose_files_unordered.push_back (pathname);
    }
    else if (extension == ".TXT" || extension == ".txt")
    {
      if (found_pose_file && extension != pose_extension)
      {
        PCL_ERROR ("Files with extension %s and %s were found in this folder! Please choose a consistent extension.\n", extension.c_str (), pose_extension.c_str ());
        return (1);
      }
      else if (!found_pose_file)
      {
        found_pose_file = true;
        binary_poses = false;
        pose_extension = extension;
      }
      pose_files_unordered.push_back (pathname);
    }
  }
  // Sort PCDS
  std::sort (pcd_files.begin (), pcd_files.end ());
  std::sort (pose_files_unordered.begin (), pose_files_unordered.end ());
  std::string pcd_prefix = getSharedPrefix(pcd_files);
  std::string pose_prefix = getSharedPrefix(pose_files_unordered);
  PCL_INFO ("Found PCD files with prefix: %s, poses with prefix: %s poses\n", pcd_prefix.c_str (), pose_prefix.c_str ());
  // For each PCD, get the pose file
  for (size_t i = 0; i < pcd_files.size (); i++)
  {
    const std::string pcd_path = pcd_files[i];
    std::string suffix = boost::filesystem::basename (boost::filesystem::path (pcd_path.substr (pcd_prefix.length())));
    std::string pose_path = pose_prefix+suffix+pose_extension;
    // Check if .transform file exists
    if (boost::filesystem::exists (pose_path))
    {
      pose_files.push_back (pose_path);
    }
    else
    {
      PCL_ERROR ("Could not find matching transform file for %s\n", pcd_path.c_str ());
      return 1;
    }
  }
  std::sort (pose_files.begin (), pose_files.end ());
  PCL_INFO ("Reading in %s pose files\n", 
            binary_poses ? "binary" : "ascii");
  std::vector<Eigen::Affine3d> poses (pose_files.size ());
  for (size_t i = 0; i < pose_files.size (); i++)
  {
    ifstream f (pose_files[i].c_str ());
    float v;
    Eigen::Matrix4d mat;
    mat (3,0) = 0; mat (3,1) = 0; mat (3,2) = 0; mat (3,3) = 1;
    for (int y = 0; y < 3; y++)
    {
      for (int x = 0; x < 4; x++)
      {
        if (binary_poses)
          f.read ((char*)&v, sizeof (float));
        else
          f >> v;
        mat (y,x) = static_cast<double> (v);
      }
    }
    f.close ();
    poses[i] = mat;
    if (invert)
      poses[i] = poses[i].inverse ();
    // Update units
    poses[i].matrix ().topRightCorner <3, 1> () *= pose_units;
    if (verbose)
    {
      std::cout << "Pose[" << i << "]" << std::endl 
                << poses[i].matrix () << std::endl;
    }
  }
  PCL_INFO ("Done!\n");

  // Begin Integration
  float tsdf_size = 12.;
  if (opts.count ("volume-size"))
    tsdf_size = opts["volume-size"].as<float> ();
  float cell_size = 0.006;
  if (opts.count ("cell-size"))
    cell_size = opts["cell-size"].as<float> ();
  int tsdf_res;
  int desired_res = tsdf_size / cell_size;
  // Snap to nearest power of 2;
  int n = 1;
  while (desired_res > n)
  {
    n *= 2;
  }
  tsdf_res = n;
  // Initialize
  cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
  if (!cloud_only)
  {
    tsdf.reset (new cpu_tsdf::TSDFVolumeOctree);
    tsdf->setGridSize (tsdf_size, tsdf_size, tsdf_size);
    PCL_INFO("Setting resolution: %d with grid size %f\n", tsdf_res, tsdf_size);
    tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
    tsdf->setImageSize (width_, height_);
    tsdf->setCameraIntrinsics (focal_length_x_, focal_length_y_, principal_point_x_, principal_point_y_);
    tsdf->setNumRandomSplts (num_random_splits);
    tsdf->setSensorDistanceBounds (min_sensor_dist, max_sensor_dist);
    tsdf->setIntegrateColor (integrate_color);
    tsdf->setDepthTruncationLimits (trunc_dist_pos, trunc_dist_neg);
    tsdf->reset ();
  }
  // Load data
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map (new pcl::PointCloud<pcl::PointXYZRGBA>);
  // Set up visualization
  pcl::visualization::PCLVisualizer::Ptr vis;
  if (visualize)
  {
     vis.reset (new pcl::visualization::PCLVisualizer);
     vis->addCoordinateSystem ();
  } 
  
  // Initialize aggregate cloud if we are just doing that instead
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregate;
  if (cloud_only)
    aggregate.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
  size_t num_frames = pcd_files.size ();
  if (opts.count ("num-frames"))
  {
    size_t user_selected_num_frames = opts["num-frames"].as<size_t> ();
    if (user_selected_num_frames <= num_frames)
    {
      num_frames = user_selected_num_frames;
    }
    else
    {
      PCL_WARN("Warning: Manually input --num-frames=%zu, but the sequence only has %zu clouds. Ignoring user specification.\n", user_selected_num_frames, num_frames);
    }
  }
  for (size_t i = 0; i < num_frames; i++)
  {
    PCL_INFO ("On frame %d / %d\n", i+1, num_frames);
    if (poses.size () <= i)
    {
      PCL_WARN ("Warning: no matching pose file found for cloud %s.\n"
                "Defaulting to identity, but unless the camera never moved, this will yield a very poor mesh!\n", 
                pcd_files[i].c_str ());
      pose_files.push_back ("not_found");
      poses.push_back (Eigen::Affine3d::Identity ());
    }
    else
    {
      PCL_INFO ("Cloud: %s, pose: %s\n", 
        pcd_files[i].c_str (), pose_files[i].c_str ());
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile (pcd_files[i], *cloud);
    if (cloud_units != 1)
    {
      for (size_t i = 0; i < cloud->size (); i++)
      {
        pcl::PointXYZRGBA &pt = cloud->at (i);
        pt.x *= cloud_units;
        pt.y *= cloud_units;
        pt.z *= cloud_units;
      }
    }
    // Remove nans
    if (zero_nans)
    {
      for (size_t j = 0; j < cloud->size (); j++)
      {
        if (cloud->at (j).x == 0 && cloud->at (j).y == 0 && cloud->at (j).z == 0)
          cloud->at (j).x = cloud->at (j).y = cloud->at (j).z = std::numeric_limits<float>::quiet_NaN ();
      }
    }
    // Transform
    if (world_frame)
      pcl::transformPointCloud (*cloud, *cloud, poses[i].inverse ());
    // Make organized
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_organized (new pcl::PointCloud<pcl::PointXYZRGBA> (width_, height_));
    if (organized)
    {
      if (cloud->height != height_ || cloud->width != width_)
      {
        PCL_ERROR ("Error: cloud %d has size %d x %d, but TSDF is initialized for %d x %d pointclouds\n", i+1, cloud->width, cloud->height, width_, height_);
        return (1);
      }
      pcl::copyPointCloud (*cloud, *cloud_organized);
    }
    else
    {
      size_t nonnan_original = 0;
      size_t nonnan_new = 0;
      float min_x = std::numeric_limits<float>::infinity ();
      float min_y = std::numeric_limits<float>::infinity ();
      float min_z = std::numeric_limits<float>::infinity ();
      float max_x = -std::numeric_limits<float>::infinity ();
      float max_y = -std::numeric_limits<float>::infinity ();
      float max_z = -std::numeric_limits<float>::infinity ();
      for (size_t j = 0; j < cloud_organized->size (); j++)
        cloud_organized->at (j).z = std::numeric_limits<float>::quiet_NaN ();
      for (size_t j = 0; j < cloud->size (); j++)
      {
        const pcl::PointXYZRGBA &pt = cloud->at (j);
        if (verbose && !pcl_isnan (pt.z))
          nonnan_original++;
        int u, v;
        if (reprojectPoint (pt, u, v))
        {
          pcl::PointXYZRGBA &pt_old = (*cloud_organized) (u, v);
          if (pcl_isnan (pt_old.z) || (pt_old.z > pt.z))
          {
            if (verbose)
            {
              if (pcl_isnan (pt_old.z))
                nonnan_new++;
              if (pt.x < min_x) min_x = pt.x;
              if (pt.y < min_y) min_y = pt.y;
              if (pt.z < min_z) min_z = pt.z;
              if (pt.x > max_x) max_x = pt.x;
              if (pt.y > max_y) max_y = pt.y;
              if (pt.z > max_z) max_z = pt.z;
            }
            pt_old = pt;
          }
        }
      }
      if (verbose)
      {
        PCL_INFO ("Reprojection yielded %d valid points, of initial %d\n", nonnan_new, nonnan_original);
        PCL_INFO ("Cloud bounds: [%f, %f], [%f, %f], [%f, %f]\n", min_x, max_x, min_y, max_y, min_z, max_x);
      }
    }
    if (visualize) // Just for visualization purposes
    {
      vis->removeAllPointClouds ();
      pcl::PointCloud<pcl::PointXYZRGBA> cloud_trans;
      pcl::transformPointCloud (*cloud_organized, cloud_trans, poses[i]);
      *map += cloud_trans;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> map_handler (map, 255, 0, 0);
      vis->addPointCloud (map, map_handler, "map");
      PCL_INFO ("Map\n");
      vis->spin ();
    }
    //Integrate
    Eigen::Affine3d  pose_rel_to_first_frame = poses[0].inverse () * poses[i];
    if (cloud_only) // Only if we're just dumping out the cloud
    {
      pcl::PointCloud<pcl::PointXYZRGBA> cloud_unorganized;
      for (size_t i = 0; i < cloud_organized->size (); i++)
      {
        if (!pcl_isnan (cloud_organized->at (i).z))
          cloud_unorganized.push_back (cloud_organized->at (i));
      }
      pcl::transformPointCloud (cloud_unorganized, cloud_unorganized, pose_rel_to_first_frame);
      *aggregate += cloud_unorganized;
      // Filter so it doesn't get too big
      if (i % 20 == 0 || i == num_frames - 1)
      {
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setLeafSize (0.01, 0.01, 0.01);
        vg.setInputCloud (aggregate);
        vg.filter (cloud_unorganized);
        *aggregate = cloud_unorganized;
      }
    }
    else
    {
      tsdf->integrateCloud (*cloud_organized, pcl::PointCloud<pcl::Normal> (), pose_rel_to_first_frame);
    }
  }
  // Save
  boost::filesystem::create_directory (out_dir);
  // If we're just saving the cloud, no need to mesh
  if (cloud_only)
  {
    pcl::io::savePCDFileBinaryCompressed (out_dir + "/cloud.pcd", *aggregate);
  }
  else
  {
    cpu_tsdf::MarchingCubesTSDFOctree mc;
    mc.setMinWeight (min_weight);
    mc.setInputTSDF (tsdf);
    if (integrate_color)
    {
      mc.setColorByRGB (true);
    }
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    mc.reconstruct (*mesh);
    if (flatten)
      flattenVertices (*mesh);
    if (cleanup)
      cleanupMesh (*mesh);
    if (visualize)
    {
      vis->removeAllPointClouds ();
      vis->addPolygonMesh (*mesh);
      vis->spin ();
    }
    PCL_INFO ("Entire pipeline took %f ms\n", tt.toc ());
    if (save_ascii)
      pcl::io::savePLYFile (out_dir + "/mesh.ply", *mesh);
    else
      pcl::io::savePLYFileBinary (out_dir + "/mesh.ply", *mesh);
    PCL_INFO ("Saved to %s/mesh.ply\n", out_dir.c_str ());
  }
}

  


