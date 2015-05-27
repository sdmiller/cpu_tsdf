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

#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

namespace cpu_tsdf
{
  class TSDFVolumeOctree : public TSDFInterface
  {
  public:
    typedef boost::shared_ptr<TSDFVolumeOctree> Ptr;
    typedef boost::shared_ptr<const TSDFVolumeOctree> ConstPtr;

    TSDFVolumeOctree ();
    ~TSDFVolumeOctree ();

    /** \brief Set the resolution of the voxel grid
     *  \param[in] xres Number of voxels in the x direction
     *  \param[in] yres Number of voxels in the y direction
     *  \param[in] zres Number of voxels in the z direction
     **/
    void
    setResolution (int xres, int yres, int zres);

    /** \brief Get the resolution of the voxel grid
     *  \param[out] xres Number of voxels in the x direction
     *  \param[out] yres Number of voxels in the y direction
     *  \param[out] zres Number of voxels in the z direction
     **/
    void
    getResolution (int &xres, int &yres, int &zres) const;

    /** \brief Set the size of the voxel grid
     *  \param[in] xsize Extent of the grid's x direction, in meters
     *  \param[in] ysize Extent of the grid's y direction, in meters
     *  \param[in] zsize Extent of the grid's z direction, in meters
     **/
    void
    setGridSize (float xsize, float ysize, float zsize);
    
    /** \brief Get the size of the voxel grid
     *  \param[out] xsize Extent of the grid's x direction, in meters
     *  \param[out] ysize Extent of the grid's y direction, in meters
     *  \param[out] zsize Extent of the grid's z direction, in meters
     */
    void
    getGridSize (float &xsize, float &ysize, float &zsize) const;

    /** \brief Set the size of the image 
     *  \param[in] width
     *  \param[in] height
     */
    void
    setImageSize (int width, int height);
    
    /** \brief Get the size of the image 
     *  \param[out] width
     *  \param[out] height
     */
    void
    getImageSize (int &width, int &height) const;

    /** \brief Set the Depth truncation parameter
     *  \param[in] max_dist Maximum distance a voxel will be set to
     */
    void
    setDepthTruncationLimits (float max_dist_pos, float max_dist_neg);

    /** \brief Get the Depth truncation parameter
     *  \return Maximum distance a voxel will be set to
     */
    void
    getDepthTruncationLimits (float &max_dist_pos, float &max_dist_neg) const;

    /** \brief Set the weight truncation parameter
     *  \param[in] max_weight Maximum weight a voxel will be set to
     */
    void
    setWeightTruncationLimit (float max_weight);

    /** \brief Get the weight truncation parameter
     *  \return Maximum weight a voxel will be set to
     */
    float
    getWeightTruncationLimit () const;


    inline void
    setGlobalTransform (const Eigen::Affine3d &trans)
    { global_transform_ = trans; }

    inline Eigen::Affine3d
    getGlobalTransform () const
    { return (global_transform_); }

    /** \brief Set the camera parameters (fx, fy, cx, cy)
      * \param[in] focal_length_x the focal length (fx)
      * \param[in] focal_length_y the focal length (fy)
      * \param[in] principal_point_x the principal point (cx)
      * \param[in] principal_point_y the principal point (cy)
      */
    void
    setCameraIntrinsics (const double focal_length_x, 
                         const double focal_length_y, 
                         const double principal_point_x,
                         const double principal_point_y);
    
    /** \brief Get the camera parameters (fx, fy, cx, cy)
      * \param[out] focal_length_x the focal length (fx)
      * \param[out] focal_length_y the focal length (fy)
      * \param[out] principal_point_x the principal point (cx)
      * \param[out] principal_point_y the principal point (cy)
      */
    void
    getCameraIntrinsics (double &focal_length_x, 
                         double &focal_length_y, 
                         double &principal_point_x,
                         double &principal_point_y) const;



    inline void
    setMaxVoxelSize (float max_cell_size_x, float max_cell_size_y, float max_cell_size_z)
    {
      max_cell_size_x_ = max_cell_size_x;
      max_cell_size_y_ = max_cell_size_y;
      max_cell_size_z_ = max_cell_size_z;
    }

    inline void
    setIntegrateColor (bool integrate_color)
    { integrate_color_ = integrate_color; }

    /** \brief Set the number of random samples per surface point
     *  the octree should split on. If you don't know what this does, 
     *  it's almost certainly best to leave it at 1*/
    inline void setNumRandomSplts (int num_random_splits)
    { num_random_splits_ = num_random_splits; }

    inline int getNumRandomSplits ()
    { return (num_random_splits_); }
    
    inline void
    setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist)
    {
      min_sensor_dist_ = min_sensor_dist;
      max_sensor_dist_ = max_sensor_dist;
    }
    
    inline void
    getSensorDistanceBounds (float &min_sensor_dist, float &max_sensor_dist) const
    {
      min_sensor_dist = min_sensor_dist_;
      max_sensor_dist = max_sensor_dist_;
    }

    /** \brief Clear everything stored in this volume and init*/
    void
    reset ();

    /** \brief Save to disk */
    void
    save (const std::string &filename) const;
    
    /** \brief Load from disk */
    void
    load (const std::string &filename);

    /** \brief Accessor to trilinearly interpolated distance */
    bool
    getFxn (const pcl::PointXYZ &pt, float &val) const;
    
    /** \brief Accessor to trilinearly interpolated derivative */
    bool
    getGradient (const pcl::PointXYZ &pt, Eigen::Vector3f &grad) const;
    
    /** \brief Accessor to Hessian */
    bool
    getHessian (const pcl::PointXYZ &pt, Eigen::Matrix3f &hessian) const;

    /** \brief Accessor to trilinearly interpolated distance and normal */
    bool
    getFxnAndGradient (const pcl::PointXYZ &pt, float &val, Eigen::Vector3f &grad) const;
    
    /** \brief Accessor to trilinearly interpolated distance and normal */
    bool
    getFxnGradientAndHessian (const pcl::PointXYZ &pt, 
                              float &val, 
                              Eigen::Vector3f &grad, 
                              Eigen::Matrix3f &hessian) const;

    /** \brief Integrate a measurement from a new cloud
     *  \param[in] cloud The cloud we are integrating
     *  \param[in] normals The normal cloud we are integrating
     *  \param[in] trans Optionally, a transform to apply to both
     *  \return True iff the integration was successful
     */
    template <typename PointT, typename NormalT> bool
    integrateCloud (const pcl::PointCloud<PointT> &cloud, 
                    const pcl::PointCloud<NormalT> &normals, 
                    const Eigen::Affine3d &trans
                      = Eigen::Affine3d::Identity ());

    /** \brief Render a view of the current surface
     *  \param[in] trans The pose of the camera relative to the TSDF
     *  \return A cloud with XYZ+Normals densely sampled along the zero crossing
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr
    renderView (const Eigen::Affine3d &trans 
                    = Eigen::Affine3d::Identity (), 
                int downsampleBy=1) const;
    
    /** \brief Render a view of the current surface
     *  \param[in] trans The pose of the camera relative to the TSDF
     *  \return A cloud with XYZ+RGB+Normals densely sampled along the zero crossing
     */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
    renderColoredView (const Eigen::Affine3d &trans 
                        = Eigen::Affine3d::Identity (), 
                       int downsampleBy=1) const;

    /** \brief Get a raytraced surface, for visualizing */
    pcl::PointCloud<pcl::Intensity>::Ptr
    getIntensityCloud (const Eigen::Affine3d &trans 
                        = Eigen::Affine3d::Identity ()) const;

    
    pcl::PointXYZ
    getVoxelCenter (size_t x, size_t y, size_t z) const;

    bool
    getVoxelIndex (float x, float y, float z, int &x_i, int &y_i, int &z_i) const;

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    getVoxelCenters (int nlevels=4) const;

    void
    getFrustumCulledVoxels (const Eigen::Affine3d &trans, std::vector<cpu_tsdf::OctreeNode::Ptr> &voxels) const;

    inline bool
    isEmpty () const
    {
      return is_empty_;
    }

    inline void
    setColorMode (const std::string &color_mode)
    {
      color_mode_ = color_mode;
    }

    void
    getOccupiedVoxelIndices (std::vector<Eigen::Vector3i> &indices) const;
    
    Octree::Ptr octree_;

    const float UNOBSERVED_VOXEL;
  protected:
    
    /** \brief Integrate a measurement from a new cloud at a single voxel
     *  \param[in] voxel The octreenode we are evaluating at
     *  \param[in] cloud The cloud we are integrating
     *  \param[in] normals The normal cloud we are integrating
     *  \param[in] trans_inv The transform which will take the voxel into the cloud's reference frame
     */
    template <typename PointT, typename NormalT> int
    updateVoxel (const cpu_tsdf::OctreeNode::Ptr &voxel,
                 const pcl::PointCloud<PointT> &cloud, 
                 const pcl::PointCloud<NormalT> &normals, 
                 const Eigen::Affine3f &trans);

    bool
    reprojectPoint (const pcl::PointXYZ &pt, int &u, int &v) const;

    float
    getTSDFValue (const Eigen::Vector3f &pt, bool* valid=NULL) const;

    float
    getTSDFValue (float x, float y, float z, bool* valid=NULL) const;

    float
    interpolateTrilinearly (const Eigen::Vector3f &pt, bool* valid=NULL) const;

    float
    interpolateTrilinearly (float x, float y, float z, bool* valid=NULL) const;

    bool
    getNeighbors (const pcl::PointXYZ &pt, std::vector<const cpu_tsdf::OctreeNode*> &neighbors, 
                  std::vector<pcl::PointXYZ> &centers) const;

    int xres_, yres_, zres_;

    float xsize_, ysize_, zsize_;

    float max_dist_pos_;

    float max_dist_neg_;

    float max_weight_;

    float min_sensor_dist_;

    float max_sensor_dist_;

    float max_cell_size_x_, max_cell_size_y_, max_cell_size_z_;

    int num_random_splits_;

    double focal_length_x_, focal_length_y_;
    double principal_point_x_, principal_point_y_;
    int image_width_, image_height_;

    bool is_empty_;

    bool weight_by_depth_;
    
    bool weight_by_variance_;

    bool integrate_color_; // track the color

    bool use_trilinear_interpolation_;

    Eigen::Affine3d global_transform_;

    std::string color_mode_;

/** \brief Get the Depth truncation parameter
 *  \return Maximum distance a voxel will be set to
 */

    // Cloud which stores the voxel centers
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers_;
    
  };
}

#include <cpu_tsdf/impl/tsdf_volume_octree.hpp>

