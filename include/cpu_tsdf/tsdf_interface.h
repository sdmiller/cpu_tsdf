/*
 * Software License Agreement
 *
 * Copyright (c) 2012-, Open Perception, Inc.
 * Author: Stephen Miller
 *  
 * All rights reserved.
 *
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <string>

namespace cpu_tsdf
{
  class TSDFInterface
  {
  public:
    typedef boost::shared_ptr<TSDFInterface> Ptr;
    typedef boost::shared_ptr<const TSDFInterface> ConstPtr;

    virtual ~TSDFInterface ()
    {}

    /** \brief Set the resolution of the voxel grid
     *  \param[in] xres Number of voxels in the x direction
     *  \param[in] yres Number of voxels in the y direction
     *  \param[in] zres Number of voxels in the z direction
     **/
    virtual void
    setResolution (int xres, int yres, int zres) = 0;

    /** \brief Get the resolution of the voxel grid
     *  \param[out] xres Number of voxels in the x direction
     *  \param[out] yres Number of voxels in the y direction
     *  \param[out] zres Number of voxels in the z direction
     **/
    virtual void
    getResolution (int &xres, int &yres, int &zres) const = 0;

    /** \brief Set the size of the voxel grid
     *  \param[in] xsize Extent of the grid's x direction, in meters
     *  \param[in] ysize Extent of the grid's y direction, in meters
     *  \param[in] zsize Extent of the grid's z direction, in meters
     **/
    virtual void
    setGridSize (float xsize, float ysize, float zsize) = 0;
    
    /** \brief Get the size of the voxel grid
     *  \param[out] xsize Extent of the grid's x direction, in meters
     *  \param[out] ysize Extent of the grid's y direction, in meters
     *  \param[out] zsize Extent of the grid's z direction, in meters
     */
    virtual void
    getGridSize (float &xsize, float &ysize, float &zsize) const = 0;

    /** \brief Set the Depth truncation parameter
     *  \param[in] max_dist Maximum distance a voxel will be set to
     */
    virtual void
    setDepthTruncationLimits (float max_dist_pos, float max_dist_neg) = 0;

    /** \brief Get the Depth truncation parameter
     *  \return Maximum distance a voxel will be set to
     */
    virtual void
    getDepthTruncationLimits (float &max_dist_pos, float &max_dist_neg) const = 0;

    /** \brief Set the weight truncation parameter
     *  \param[in] max_weight Maximum weight a voxel will be set to
     */
    virtual void
    setWeightTruncationLimit (float max_weight) = 0;

    /** \brief Get the weight truncation parameter
     *  \return Maximum weight a voxel will be set to
     */
    virtual float
    getWeightTruncationLimit () const = 0;
    
    virtual void
    setGlobalTransform (const Eigen::Affine3d &trans) = 0;

    virtual Eigen::Affine3d
    getGlobalTransform () const = 0;

    virtual void
    setSensorDistanceBounds (float min_sensor_dist, float max_sensor_dist) = 0;
    
    virtual void
    getSensorDistanceBounds (float &min_sensor_dist, float &max_sensor_dist) const = 0;

    /** \brief Save in ascii format to disk */
    virtual void
    save (const std::string &filename) const = 0;
    
    /** \brief Load from disk */
    virtual void
    load (const std::string &filename) = 0;

    static TSDFInterface::Ptr
    instantiateFromFile (const std::string &filename);
    
    /** \brief Accessor to trilinearly interpolated distance */
    virtual bool
    getFxn (const pcl::PointXYZ &pt, float &val) const = 0;
    
    /** \brief Accessor to trilinearly interpolated derivative */
    virtual bool
    getGradient (const pcl::PointXYZ &pt, Eigen::Vector3f &grad) const = 0;

    /** \brief Accessor to Hessian */
    virtual bool
    getHessian (const pcl::PointXYZ &pt, Eigen::Matrix3f &hessian) const = 0;

    /** \brief Accessor to trilinearly interpolated distance and normal */
    virtual bool
    getFxnAndGradient (const pcl::PointXYZ &pt, float &val, Eigen::Vector3f &grad) const
    {
      return (getFxn (pt, val) && getGradient (pt, grad));
    }
    
    /** \brief Accessor to trilinearly interpolated distance and normal */
    virtual bool
    getFxnGradientAndHessian (const pcl::PointXYZ &pt, 
                              float &val, 
                              Eigen::Vector3f &grad, 
                              Eigen::Matrix3f &hessian) const
    {
      return (getFxn (pt, val) && getGradient (pt, grad) && getHessian (pt, hessian));
    }


  };
}



