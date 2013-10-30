//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    tCluster.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 * \brief   Contains tCluster
 *
 * \b tCluster
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__cluster_analysis__tCluster_h__
#define __rrlib__model_fitting__cluster_analysis__tCluster_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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
// Class declaration
//----------------------------------------------------------------------
template <typename TSample>
class tCluster
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef geometry::tBoundingBox<TSample::cDIMENSION, typename TSample::tElement> tBoundingBox;

  typedef std::function <typename TSample::tElement(const TSample &a, const TSample &b)> tMetric;

  tCluster(const TSample &center);

  /*!
   * \brief Get the center of this cluster (read-only)
   *
   * \return The coordinates of the centroid of this cluster
   */
  inline const TSample &Center() const;

  /*!
   * \brief Get the measurements belonging to this cluster (read-only)
   *
   * \return A reference to the measurements-list of this cluster
   */
  inline const std::vector<TSample> &Samples() const;

  inline const tBoundingBox &Bounds() const;

  inline typename TSample::tElement SumOfNorms() const;

  void Update(const TSample &center, double weight = 1.0);

  bool ApplyUpdates(tMetric metric);

  void AddSample(const TSample &sample);

  void ComputeSumOfNorms(tMetric metric);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TSample center;
  typename std::vector<TSample> samples;
  tBoundingBox bounds;
  typename TSample::tElement sum_of_norms;

  TSample new_center;
  double normalization_factor;

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/model_fitting/cluster_analysis/tCluster.hpp"

#endif
