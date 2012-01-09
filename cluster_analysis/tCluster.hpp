//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    tCluster.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace model_fitting
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tCluster constructors
//----------------------------------------------------------------------
template <typename TSample>
tCluster<TSample>::tCluster(const TSample &center)
    : center(center),
    sum_of_norms(0),
    normalization_factor(0)
{}

//----------------------------------------------------------------------
// tCluster Center
//----------------------------------------------------------------------
template <typename TSample>
const TSample &tCluster<TSample>::Center() const
{
  return this->center;
}

//----------------------------------------------------------------------
// tCluster Samples
//----------------------------------------------------------------------
template <typename TSample>
inline const std::vector<TSample> &tCluster<TSample>::Samples() const
{
  return this->samples;
}

//----------------------------------------------------------------------
// tCluster Bounds
//----------------------------------------------------------------------
template <typename TSample>
const typename tCluster<TSample>::tBoundingBox &tCluster<TSample>::Bounds() const
{
  return this->bounds;
}

//----------------------------------------------------------------------
// tCluster SumOfNorms
//----------------------------------------------------------------------
template <typename TSample>
typename TSample::tElement tCluster<TSample>::SumOfNorms() const
{
  return this->sum_of_norms;
}

//----------------------------------------------------------------------
// tCluster Update
//----------------------------------------------------------------------
template <typename TSample>
void tCluster<TSample>::Update(const TSample &sample, double weight)
{
  this->new_center += sample * weight;
  this->normalization_factor += weight;
}

//----------------------------------------------------------------------
// tCluster ApplyUpdates
//----------------------------------------------------------------------
template <typename TSample>
bool tCluster<TSample>::ApplyUpdates(tMetric metric)
{
  if (this->normalization_factor == 0.0)
  {
    return false;
  }

  this->new_center *= 1.0 / this->normalization_factor;
  bool update_is_noticable = metric(this->center, this->new_center) > 1E-6;

  this->center = this->new_center;
  this->new_center = TSample::Zero();
  this->normalization_factor = 0.0;

  return update_is_noticable;
}

//----------------------------------------------------------------------
// tCluster AddSample
//----------------------------------------------------------------------
template <typename TSample>
void tCluster<TSample>::AddSample(const TSample &sample)
{
  this->bounds.Add(sample);
  this->samples.push_back(sample);
}

//----------------------------------------------------------------------
// tCluster ComputeSumOfNorms
//----------------------------------------------------------------------
template <typename TSample>
void tCluster<TSample>::ComputeSumOfNorms(tMetric metric)
{
  this->sum_of_norms = 0;
  for (auto it = this->samples.begin(); it != this->samples.end(); ++it)
  {
    typename TSample::tElement distance = metric(*it, this->Center());
    this->sum_of_norms += distance * distance;
  }
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
