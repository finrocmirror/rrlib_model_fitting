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
/*!\file    tClustering.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 * \brief   Contains tClustering
 *
 * \b tClustering
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__cluster_analysis__tClustering_h__
#define __rrlib__model_fitting__cluster_analysis__tClustering_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/cluster_analysis/tCluster.h"

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
class tClustering
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TSample tSample;

  typedef model_fitting::tCluster<TSample> tCluster;

  typedef typename TSample::tMetric tMetric;

  static const tMetric cDEFAULT_METRIC;

  virtual ~tClustering() = 0;

  inline const std::vector<tCluster> &Clusters() const;

  /*!
   * \brief This method sorts the calculated clusters by decreasing size
   */
  inline void Sort();

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  std::vector<tCluster> clusters;

  /*!
   * \brief Get the nearest cluster for one measurement
   *
   * \param measurement   The point
   * \param metric        The functor which computes an appropriate metric
   *
   * \return The cluster which has the smallest distance to the given measurement in terms of the given metric
   */
  size_t GetNearestClusterID(const TSample &sample, tMetric metric);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/model_fitting/cluster_analysis/tClustering.hpp"

#endif
