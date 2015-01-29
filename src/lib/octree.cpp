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


#include <cpu_tsdf/octree.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/pcl_macros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <stdint.h>
#include <cmath>
#include <fstream>
#include <iostream>

void
cpu_tsdf::OctreeNode::getCenter (float &x, float &y, float &z) const
{
  x = ctr_x_;
  y = ctr_y_;
  z = ctr_z_;
};

void
cpu_tsdf::OctreeNode::getSize (float &size_x, float &size_y, float &size_z) const
{
  size_x = size_;
  size_y = size_;
  size_z = size_;
}

float
cpu_tsdf::OctreeNode::getMaxSize () const
{
  return (std::sqrt (3) * size_);
}

float
cpu_tsdf::OctreeNode::getMinSize () const
{
  return (size_);
}

bool
cpu_tsdf::OctreeNode::hasChildren () const
{
  return (!children_.empty ());
}

std::vector<cpu_tsdf::OctreeNode::Ptr>&
cpu_tsdf::OctreeNode::getChildren ()
{
  return children_;
}

const std::vector<cpu_tsdf::OctreeNode::Ptr>&
cpu_tsdf::OctreeNode::getChildren () const
{
  return children_;
}

void
cpu_tsdf::OctreeNode::getLeaves (std::vector<cpu_tsdf::OctreeNode::Ptr> &leaves, int num_levels)
{
  for (size_t i = 0; i < children_.size (); i++)
  {
    const OctreeNode::Ptr &child = children_[i];
    if (child->hasChildren () && num_levels != 0)
      child->getLeaves (leaves, num_levels - 1);
    else
      leaves.push_back (child);
  }
}

// Get the voxel which contains this point
cpu_tsdf::OctreeNode*
cpu_tsdf::OctreeNode::getContainingVoxel (float x, float y, float z, float min_size)
{
  if (!hasChildren () || (min_size > 0 && size_ <= min_size))
    return (this);
  else
  {
    return children_[((x-ctr_x_) > 0) * 4 + ((y-ctr_y_) > 0) * 2 + (z - ctr_z_ > 0)]->getContainingVoxel (x, y, z, min_size);
  }
}

// Get the voxel which contains this point
const cpu_tsdf::OctreeNode*
cpu_tsdf::OctreeNode::getContainingVoxel (float x, float y, float z, float min_size) const
{
  if (!hasChildren () || (min_size > 0 && size_ <= min_size))
    return (this);
  else
  {
    return children_[((x-ctr_x_) > 0) * 4 + ((y-ctr_y_) > 0) * 2 + (z - ctr_z_ > 0)]->getContainingVoxel (x, y, z, min_size);
  }
}


bool
cpu_tsdf::OctreeNode::getData (float &d, float &w) const
{
  d = d_;
  w = w_;
  return (true);
}

bool
cpu_tsdf::OctreeNode::setData (float d, float w)
{
  d_ = d;
  w_ = w;
  return (true);
}

bool
cpu_tsdf::OctreeNode::addObservation (float d_new, float w_new, float max_weight)
{
  float d_old = d_;
  d_ = (d_ * w_ + d_new * w_new) / (w_ + w_new);
  w_ += w_new;
  if (w_ > max_weight)
    w_ = max_weight;
  M_ += w_new * (d_new - d_) * (d_new - d_old);
  ++nsample_;
  return (true);
}


bool
cpu_tsdf::OctreeNode::addObservation (float d_new, float w_new, float max_weight, 
                uint8_t r, uint8_t g, uint8_t b)
{
  return (addObservation (d_new, w_new, max_weight));
}

bool
cpu_tsdf::OctreeNode::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const
{
  r = g = b = 127;
  return (false);
}

cpu_tsdf::OctreeNode*
cpu_tsdf::OctreeNode::instantiateNode (float x, float y, float z, float sx, float sy, float sz)
{
  return (new OctreeNode (x, y, z, sx, sy, sz));
}

std::string
cpu_tsdf::OctreeNode::getTypeString ()
{
  return ("NOCOLOR");
}

cpu_tsdf::OctreeNode*
cpu_tsdf::OctreeNode::instantiateByTypeString (const std::string &str)
{
  if (str == "NOCOLOR")
    return (new OctreeNode);
  else if (str == "RGB")
    return (new RGBNode);
  else if (str == "RGBNormalized")
    return (new RGBNormalized);
  else if (str == "LAB")
    return (new LABNode);
  // Handle more examples
  PCL_ERROR ("[cpu_tsdf::OctreeNode::instantiateByTypeString] Requested invalid type string %s\n", str.c_str ());
  return (NULL);
}

cpu_tsdf::OctreeNode*
cpu_tsdf::OctreeNode::instantiateByTypeString (const std::string &str, 
                         float x, float y, float z, float sx, float sy, float sz)
{
  cpu_tsdf::OctreeNode* empty_node = instantiateByTypeString (str);
  cpu_tsdf::OctreeNode* node = empty_node->instantiateNode (x, y, z, sx, sy, sz);
  delete empty_node;
  return (node);
}

void
cpu_tsdf::OctreeNode::updateAverage ()
{
  if (children_.empty ())
    return;

  float d_avg = 0;
  float w_avg = 0;
  int ngood = 0;
  for (size_t i = 0; i < children_.size (); ++i)
  {
    children_[i]->updateAverage ();
    if (children_[i]->w_ > 0)
    {
      d_avg += children_[i]->d_;
      w_avg += children_[i]->w_;
      ++ngood;
    }
  }
  if (ngood > 0)
  {
    d_ = d_avg/ngood;
    w_ = w_avg/ngood;
  }
}

std::vector<cpu_tsdf::OctreeNode::Ptr>& 
cpu_tsdf::OctreeNode::split ()
{
  children_.resize (8);
  //float off_x = size_x_ / 4;
  //float off_y = size_y_ / 4;
  //float off_z = size_z_ / 4;
  float off_x = size_ / 4;
  float off_y = size_ / 4;
  float off_z = size_ / 4;
  float newsize_x = size_ / 2;
  float newsize_y = size_ / 2;
  float newsize_z = size_ / 2;
  children_[0].reset (instantiateNode (ctr_x_-off_x, ctr_y_-off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
  children_[1].reset (instantiateNode (ctr_x_-off_x, ctr_y_-off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
  children_[2].reset (instantiateNode (ctr_x_-off_x, ctr_y_+off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
  children_[3].reset (instantiateNode (ctr_x_-off_x, ctr_y_+off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
  children_[4].reset (instantiateNode (ctr_x_+off_x, ctr_y_-off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
  children_[5].reset (instantiateNode (ctr_x_+off_x, ctr_y_-off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
  children_[6].reset (instantiateNode (ctr_x_+off_x, ctr_y_+off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
  children_[7].reset (instantiateNode (ctr_x_+off_x, ctr_y_+off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
  return (children_);
}


void
cpu_tsdf::OctreeNode::splitRecursive (int num_left)
{
  if (num_left <= 0)
    return;
  split ();
  for (size_t i = 0; i < children_.size (); i++)
  {
    children_[i]->splitRecursive (num_left-1);
  }
}

float
cpu_tsdf::OctreeNode::getVariance () const
{
  if (nsample_ < 5)
    return (std::numeric_limits<float>::infinity ());
  return ((M_/w_)*(nsample_/(nsample_-1)));
}

void
cpu_tsdf::OctreeNode::serialize (std::ostream &f) const
{
  f.write ((char*)&d_, sizeof (float));
  f.write ((char*)&w_, sizeof (float));
  f.write ((char*)&ctr_x_, sizeof (float));
  f.write ((char*)&ctr_y_, sizeof (float));
  f.write ((char*)&ctr_z_, sizeof (float));
  f.write ((char*)&size_, sizeof (float));
  f.write ((char*)&M_, sizeof (float));
  f.write ((char*)&nsample_, sizeof (int));
  size_t nchild = children_.size ();
  f.write ((char*)&nchild, sizeof (size_t));
  for (size_t i = 0; i < nchild; ++i)
    children_[i]->serialize (f);
}

void
cpu_tsdf::OctreeNode::deserialize (std::istream &f)
{
  f.read ((char*)&d_, sizeof (float));
  f.read ((char*)&w_, sizeof (float));
  f.read ((char*)&ctr_x_, sizeof (float));
  f.read ((char*)&ctr_y_, sizeof (float));
  f.read ((char*)&ctr_z_, sizeof (float));
  f.read ((char*)&size_, sizeof (float));
  f.read ((char*)&M_, sizeof (float));
  f.read ((char*)&nsample_, sizeof (int));
  size_t nchild;
  f.read ((char*)&nchild, sizeof (size_t));
  children_.resize (nchild);
  for (size_t i = 0; i < nchild; ++i)
  {
    children_[i].reset (instantiateNode (0, 0, 0, 0, 0, 0));
    children_[i]->deserialize (f);
  }
}

// RGBNode
bool
cpu_tsdf::RGBNode::addObservation (float d_new, float w_new, float max_weight, 
                uint8_t r, uint8_t g, uint8_t b)
{
  float wsum = w_ + w_new;
  r_ = static_cast<uint8_t> ( (w_*r_ + w_new*r) / wsum);
  g_ = static_cast<uint8_t> ( (w_*g_ + w_new*g) / wsum);
  b_ = static_cast<uint8_t> ( (w_*b_ + w_new*b) / wsum);
  return (OctreeNode::addObservation (d_new, w_new, max_weight));
}

bool
cpu_tsdf::RGBNode::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const
{
  r = r_;
  g = g_;
  b = b_;
  return (true);
}

cpu_tsdf::OctreeNode*
cpu_tsdf::RGBNode::instantiateNode (float x, float y, float z, float sx, float sy, float sz)
{
  return (new RGBNode (x, y, z, sx, sy, sz));
}

std::string
cpu_tsdf::RGBNode::getTypeString ()
{
  return ("RGB");
}

void
cpu_tsdf::RGBNode::serialize (std::ostream &f) const
{
  f.write ((char*)&r_, sizeof (uint8_t));
  f.write ((char*)&g_, sizeof (uint8_t));
  f.write ((char*)&b_, sizeof (uint8_t));
  OctreeNode::serialize (f);
}

void
cpu_tsdf::RGBNode::deserialize (std::istream &f)
{
  f.read ((char*)&r_, sizeof (uint8_t));
  f.read ((char*)&g_, sizeof (uint8_t));
  f.read ((char*)&b_, sizeof (uint8_t));
  OctreeNode::deserialize (f);
}

// RGBNormalized
bool
cpu_tsdf::RGBNormalized::addObservation (float d_new, float w_new, float max_weight, 
                uint8_t r, uint8_t g, uint8_t b)
{
  float wsum = w_ + w_new;
  float i = std::sqrt ((float)r * (float)r + (float)g*(float)g + (float)b*(float)b);
  float r_f = r / i;
  float g_f = g / i;
  float b_f = b / i;
  r_n_ = (w_*r_n_ + w_new*r_f) / wsum;
  g_n_ = (w_*g_n_ + w_new*g_f) / wsum;
  b_n_ = (w_*b_n_ + w_new*b_f) / wsum;
  i_ = (w_*i_ + w_new*i) / wsum;
  return (OctreeNode::addObservation (d_new, w_new, max_weight));
}

bool
cpu_tsdf::RGBNormalized::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const
{
  r = r_n_ * i_;
  g = g_n_ * i_;
  b = b_n_ * i_;
  return (true);
}

cpu_tsdf::OctreeNode*
cpu_tsdf::RGBNormalized::instantiateNode (float x, float y, float z, float sx, float sy, float sz)
{
  return (new RGBNormalized (x, y, z, sx, sy, sz));
}

std::string
cpu_tsdf::RGBNormalized::getTypeString ()
{
  return ("RGBNormalized");
}

void
cpu_tsdf::RGBNormalized::serialize (std::ostream &f) const
{
  f.write ((char*)&r_n_, sizeof (uint8_t));
  f.write ((char*)&g_n_, sizeof (uint8_t));
  f.write ((char*)&b_n_, sizeof (uint8_t));
  f.write ((char*)&i_, sizeof (uint8_t));
  OctreeNode::serialize (f);
}

void
cpu_tsdf::RGBNormalized::deserialize (std::istream &f)
{
  f.read ((char*)&r_n_, sizeof (uint8_t));
  f.read ((char*)&g_n_, sizeof (uint8_t));
  f.read ((char*)&b_n_, sizeof (uint8_t));
  f.read ((char*)&i_, sizeof (uint8_t));
  OctreeNode::deserialize (f);
}

void
cpu_tsdf::RGB2LAB (uint8_t r, uint8_t g, uint8_t b, 
             float &L, float &A, float &B)
{
  // RGB to XYZ
  float rf = (static_cast<float> (r) / 255.);
  float gf = (static_cast<float> (g) / 255.);
  float bf = (static_cast<float> (b) / 255.);
  if (rf > 0.0405) 
    rf = std::pow(((rf + 0.055) / 1.055), 2.4);
  else
    rf /= 12.92;
  if (gf > 0.0405) 
    gf = std::pow(((gf + 0.055) / 1.055), 2.4);
  else
    gf /= 12.92;
  if (bf > 0.0405) 
    bf = std::pow(((bf + 0.055) / 1.055), 2.4);
  else
    bf /= 12.92;
  rf *= 100;
  gf *= 100;
  bf *= 100;
  float X = rf * 0.4124 + gf * 0.3576 + bf * 0.1805;
  float Y = rf * 0.2126 + gf * 0.7152 + bf * 0.0722;
  float Z = rf * 0.0193 + gf * 0.1192 + bf * 0.9505;
  // XYZ to LAB
  X /= 95.047;
  Y /= 100.;
  Z /= 108.883;
  if (X > 0.008856)
    X = std::pow (static_cast<double> (X), 1/3.);
  else
    X = 7.787 * X + (16 / 116.);
  if (Y > 0.008856)
    Y = std::pow (static_cast<double> (Y), 1/3.);
  else
    Y = 7.787 * Y + (16 / 116.);
  if (Z > 0.008856)
    Z = std::pow (static_cast<double> (Z), 1/3.);
  else
    Z = 7.787 * Z + (16 / 116.);
  L = (116 * Y) - 16;
  A = 500 * (X - Y);
  B = 200 * (Y - Z);
}

void
cpu_tsdf::LAB2RGB (float L, float A, float B, 
             uint8_t &r, uint8_t &g, uint8_t &b)
{
  // LAB to XYZ
  float Y = (L + 16) / 116.;
  float X = A / 500. + Y;
  float Z = Y - (B / 200.);
  if (std::pow (X, 3) > 0.008856)
    X = std::pow (X, 3);
  else
    X = (X - 16 / 116.) / 7.787;
  if (std::pow (Y, 3) > 0.008856)
    Y = std::pow (Y, 3);
  else
    Y = (Y - 16 / 116.) / 7.787;
  if (std::pow (Z, 3) > 0.008856)
    Z = std::pow (Z, 3);
  else
    Z = (Z - 16 / 116.) / 7.787;
  X *= 95.047;
  Y *= 100.;
  Z *= 108.883;
  // XYZ to RGB
  X /= 100;
  Y /= 100;
  Z /= 100;
  float rf = X * +3.2406 + Y * -1.5372 + Z * -0.4986;
  float gf = X * -0.9689 + Y * +1.8758 + Z * +0.0415;
  float bf = X * +0.0557 + Y * -0.2040 + Z * +1.0570;
  if (rf > 0.0031308)
    rf = 1.055 * std::pow (static_cast<double> (rf), 1. / 2.4) - 0.055;
  else
    rf *= 12.92;
  if (gf > 0.0031308)
    gf = 1.055 * std::pow (static_cast<double> (gf), 1. / 2.4) - 0.055;
  else
    gf *= 12.92;
  if (bf > 0.0031308)
    bf = 1.055 * std::pow (static_cast<double> (bf), 1. / 2.4) - 0.055;
  else
    bf *= 12.92;
  r = static_cast<uint8_t> (rf * 255);
  g = static_cast<uint8_t> (gf * 255);
  b = static_cast<uint8_t> (bf * 255);
}

// LABNode
bool
cpu_tsdf::LABNode::addObservation (float d_new, float w_new, float max_weight, 
                uint8_t r, uint8_t g, uint8_t b)
{
  float wsum = w_ + w_new;
  float L_new, A_new, B_new;
  RGB2LAB (r, g, b, L_new, A_new, B_new);
  uint8_t r_reconv, g_reconv, b_reconv;
  LAB2RGB (L_new, A_new, B_new, r_reconv, g_reconv, b_reconv);
  L_ = (w_*L_ + w_new*L_new) / wsum;
  A_ = (w_*A_ + w_new*A_new) / wsum;
  B_ = (w_*B_ + w_new*B_new) / wsum;
  return (OctreeNode::addObservation (d_new, w_new, max_weight));
}

bool
cpu_tsdf::LABNode::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const
{
  LAB2RGB (L_, A_, B_, r, g, b);
  return (true);
}

cpu_tsdf::OctreeNode*
cpu_tsdf::LABNode::instantiateNode (float x, float y, float z, float sx, float sy, float sz)
{
  return (new LABNode (x, y, z, sx, sy, sz));
}

std::string
cpu_tsdf::LABNode::getTypeString ()
{
  return ("LAB");
}

void
cpu_tsdf::LABNode::serialize (std::ostream &f) const
{
  f.write ((char*)&L_, sizeof (uint8_t));
  f.write ((char*)&A_, sizeof (uint8_t));
  f.write ((char*)&B_, sizeof (uint8_t));
  OctreeNode::serialize (f);
}

void
cpu_tsdf::LABNode::deserialize (std::istream &f)
{
  f.read ((char*)&L_, sizeof (uint8_t));
  f.read ((char*)&A_, sizeof (uint8_t));
  f.read ((char*)&B_, sizeof (uint8_t));
  OctreeNode::deserialize (f);
}

// Octree
void
cpu_tsdf::Octree::init (int num_splits)
{
  // Starts with just one root node
  root_.reset (OctreeNode::instantiateByTypeString 
      (voxel_type_, 0, 0, 0, size_x_, size_y_, size_z_));
  root_->splitRecursive (num_splits);
}

void
cpu_tsdf::Octree::init (float max_size_x, float max_size_y, float max_size_z)
{
  int desired_res = std::max (size_x_/max_size_x, std::max (size_y_ / max_size_y, size_z_/max_size_z));
  int num_levels = std::ceil (std::log (desired_res) / std::log (2));
  init (num_levels);
}

cpu_tsdf::OctreeNode::Ptr&
cpu_tsdf::Octree::getRoot ()
{
  return root_;
}

void 
cpu_tsdf::Octree::getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels) const
{
  // Recursively find leaf nodes
  pcl::console::TicToc tt;
  tt.tic ();
  if (num_levels == 0)
    leaves.push_back (root_);
  else
    root_->getLeaves (leaves, num_levels-1);
}

void
cpu_tsdf::Octree::getLeaves (std::vector<OctreeNode::Ptr> &leaves, float max_size_x, float max_size_y, float max_size_z) const
{
  int desired_res = std::max (size_x_/max_size_x, std::max (size_y_ / max_size_y, size_z_/max_size_z));
  int num_levels = std::ceil (std::log (desired_res) / std::log (2));
  getLeaves (leaves, num_levels);
}

// Get the voxel which contains this point
const cpu_tsdf::OctreeNode*
cpu_tsdf::Octree::getContainingVoxel (float x, float y, float z, float min_size) const
{
  if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs (y) > size_y_/2 || std::fabs (z) > size_z_/2)
    return (NULL);
  return (root_->getContainingVoxel (x, y, z, min_size));
}

// Get the voxel which contains this point
cpu_tsdf::OctreeNode*
cpu_tsdf::Octree::getContainingVoxel (float x, float y, float z, float min_size)
{
  if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs (y) > size_y_/2 || std::fabs (z) > size_z_/2)
    return (NULL);
  return (root_->getContainingVoxel (x, y, z, min_size));
}

void
cpu_tsdf::Octree::serialize (std::ostream &f) const
{
  f << root_->getTypeString () << std::endl;
  f << "#OCTREEBINARY" << std::endl;
  f.write ((char*)&res_x_, sizeof (size_t));
  f.write ((char*)&res_y_, sizeof (size_t));
  f.write ((char*)&res_z_, sizeof (size_t));
  f.write ((char*)&size_x_, sizeof (float));
  f.write ((char*)&size_y_, sizeof (float));
  f.write ((char*)&size_z_, sizeof (float));
  root_->serialize (f);
}

void
cpu_tsdf::Octree::deserialize (std::istream &f)
{
  std::string root_type;
  f >> root_type;
  root_.reset (OctreeNode::instantiateByTypeString (root_type));
  char tmp[1024];
  do
  {
    f.getline (tmp, 1024);
  }
  while (!(tmp[0] == '#' && tmp[1] == 'O'));
  f.read ((char*)&res_x_, sizeof (size_t));
  f.read ((char*)&res_y_, sizeof (size_t));
  f.read ((char*)&res_z_, sizeof (size_t));
  f.read ((char*)&size_x_, sizeof (float));
  f.read ((char*)&size_y_, sizeof (float));
  f.read ((char*)&size_z_, sizeof (float));
  root_->deserialize (f);
}

