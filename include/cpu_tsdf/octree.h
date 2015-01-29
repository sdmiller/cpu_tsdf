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
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/pcl_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <stdint.h>
#include <cmath>
#include <fstream>
#include <iostream>

namespace cpu_tsdf
{

  class OctreeNode
  {
  public:
    typedef boost::shared_ptr<OctreeNode> Ptr;
    typedef boost::shared_ptr<const OctreeNode> ConstPtr;

    OctreeNode () {}

    OctreeNode (float x, float y, float z, float size_x, float size_y, float size_z)
      : ctr_x_ (x)
      , ctr_y_ (y)
      , ctr_z_ (z)
      , size_ (size_x)
      //, size_x_ (size_x) 
      //, size_y_ (size_y)
      //, size_z_ (size_z)
      , d_ (-1)
      , w_ (0)
      , M_ (0)
      , nsample_ (0)
    {
    }

    virtual ~OctreeNode () {}

    void
    getCenter (float &x, float &y, float &z) const;
    
    void
    getSize (float &size_x, float &size_y, float &size_z) const;

    float
    getMaxSize () const;
    
    float
    getMinSize () const;

    bool
    hasChildren () const;

    std::vector<OctreeNode::Ptr>&
    getChildren ();
    
    const std::vector<OctreeNode::Ptr>&
    getChildren () const;

    void
    getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels);

    // Get the voxel which contains this point
    cpu_tsdf::OctreeNode*
    getContainingVoxel (float x, float y, float z, float min_size=-1);

    // Get the voxel which contains this point
    const cpu_tsdf::OctreeNode*
    getContainingVoxel (float x, float y, float z, float min_size=-1) const;
    
    

    bool
    getData (float &d, float &w) const;

    bool
    setData (float d, float w);

    virtual bool
    addObservation (float d_new, float w_new, float max_weight);
    
    
    virtual bool
    addObservation (float d_new, float w_new, float max_weight, 
                    uint8_t r, uint8_t g, uint8_t b);

    virtual bool
    getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;
    
    virtual cpu_tsdf::OctreeNode*
    instantiateNode (float x, float y, float z, float sx, float sy, float sz);

    virtual std::string
    getTypeString ();

    static cpu_tsdf::OctreeNode*
    instantiateByTypeString (const std::string &str);

    static cpu_tsdf::OctreeNode*
    instantiateByTypeString (const std::string &str, 
                             float x, float y, float z, float sx, float sy, float sz);

    void
    updateAverage ();

    std::vector<OctreeNode::Ptr>& split ();


    void
    splitRecursive (int num_left);

    float
    getVariance () const;

    virtual void
    serialize (std::ostream &f) const;

    virtual void
    deserialize (std::istream &f);

    float d_;
    float w_;
    float M_;
    int nsample_;
  protected:
    float ctr_x_;
    float ctr_y_;
    float ctr_z_;
    float size_;
    std::vector<OctreeNode::Ptr> children_;
  };

  class RGBNode : public OctreeNode
  {
  public:
    RGBNode (float x, float y, float z, float size_x, float size_y, float size_z):
      r_ (0), 
      g_ (0), 
      b_ (0), 
      OctreeNode (x, y, z, size_x, size_y, size_x)
    {}

    RGBNode ():
      OctreeNode ()
    {}

    virtual ~RGBNode () {}

    virtual bool
    addObservation (float d_new, float w_new, float max_weight, 
                    uint8_t r, uint8_t g, uint8_t b);

    virtual bool
    getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;

    virtual cpu_tsdf::OctreeNode*
    instantiateNode (float x, float y, float z, float sx, float sy, float sz);

    virtual std::string
    getTypeString ();

    virtual void
    serialize (std::ostream &f) const;

    virtual void
    deserialize (std::istream &f);

    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
  };
  
  class RGBNormalized : public OctreeNode
  {
  public:
    RGBNormalized (float x, float y, float z, float size_x, float size_y, float size_z):
      r_n_ (0), 
      g_n_ (0), 
      b_n_ (0),
      i_ (0),
      OctreeNode (x, y, z, size_x, size_y, size_x)
    {}

    RGBNormalized ():
      OctreeNode ()
    {}

    virtual ~RGBNormalized () {}

    bool
    addObservation (float d_new, float w_new, float max_weight, 
                    uint8_t r, uint8_t g, uint8_t b);

    bool
    getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;

    cpu_tsdf::OctreeNode*
    instantiateNode (float x, float y, float z, float sx, float sy, float sz);

    std::string
    getTypeString ();

    void
    serialize (std::ostream &f) const;

    void
    deserialize (std::istream &f);

    float r_n_;
    float g_n_;
    float b_n_;
    float i_;
  };

  void
  RGB2LAB (uint8_t r, uint8_t g, uint8_t b, 
           float &L, float &A, float &B);
  
  void
  LAB2RGB (float L, float A, float B,
           uint8_t &r, uint8_t &g, uint8_t &b); 
  
  class LABNode : public OctreeNode
  {
  public:
    LABNode (float x, float y, float z, float size_x, float size_y, float size_z):
      L_ (0), 
      A_ (0), 
      B_ (0),
      OctreeNode (x, y, z, size_x, size_y, size_x)
    {}

    LABNode ():
      OctreeNode ()
    {}

    virtual ~LABNode () {}

    bool
    addObservation (float d_new, float w_new, float max_weight, 
                    uint8_t r, uint8_t g, uint8_t b);

    bool
    getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const;

    cpu_tsdf::OctreeNode*
    instantiateNode (float x, float y, float z, float sx, float sy, float sz);

    std::string
    getTypeString ();

    void
    serialize (std::ostream &f) const;

    void
    deserialize (std::istream &f);

    float L_;
    float A_;
    float B_;
  };

  class Octree
  {
  public:
    typedef boost::shared_ptr<Octree> Ptr;
    typedef boost::shared_ptr<const Octree> ConstPtr;
    Octree (size_t res_x, size_t res_y, size_t res_z, float size_x, float size_y, float size_z, const std::string voxel_type = "NOCOLOR"):
      res_x_ (res_x), 
      res_y_ (res_y), 
      res_z_ (res_z), 
      size_x_ (size_x), 
      size_y_ (size_y), 
      size_z_ (size_z),
      voxel_type_ (voxel_type)
    {

    }

    Octree () 
    {
    }

    void
    init (int num_splits=0);

    void
    init (float max_size_x, float max_size_y, float max_size_z);

    OctreeNode::Ptr&
    getRoot ();
    
    void 
    getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels = -1) const;
    
    void
    getLeaves (std::vector<OctreeNode::Ptr> &leaves, float max_size_x, float max_size_y, float max_size_z) const;
    
    // Get the voxel which contains this point
    const cpu_tsdf::OctreeNode*
    getContainingVoxel (float x, float y, float z, float min_size=-1) const;
    
    // Get the voxel which contains this point
    cpu_tsdf::OctreeNode*
    getContainingVoxel (float x, float y, float z, float min_size=-1);
    
    void
    serialize (std::ostream &f) const;

    void
    deserialize (std::istream &f);

  protected:
    size_t res_x_, res_y_, res_z_;
    float size_x_, size_y_, size_z_;
    std::string voxel_type_;

    OctreeNode::Ptr root_;
    mutable std::vector<OctreeNode::Ptr> leaves_cached_;
  };
}
