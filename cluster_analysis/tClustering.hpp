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
/*!\file    tClustering.hpp
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
template <typename TSample>
const typename tClustering<TSample>::tMetric tClustering<TSample>::cDEFAULT_METRIC = TSample::cEUCLIDEAN_DISTANCE;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tClustering destructor
//----------------------------------------------------------------------
template <typename TSample>
tClustering<TSample>::~tClustering()
{}

//----------------------------------------------------------------------
// tClustering Clusters
//----------------------------------------------------------------------
template <typename TSample>
const std::vector<typename tClustering<TSample>::tCluster> &tClustering<TSample>::Clusters() const
{
  return this->clusters;
}

//----------------------------------------------------------------------
// tClustering Sort
//----------------------------------------------------------------------
template <typename TSample>
void tClustering<TSample>::Sort()
{
  std::sort(this->clusters.begin(), this->clusters.end(), [this](const tCluster &a, const tCluster &b)
  {
    return a.Samples().size() > b.Samples().size();
  });
}

//----------------------------------------------------------------------
// tClustering GetNearestClusterID
//----------------------------------------------------------------------
template <typename TSample>
size_t tClustering<TSample>::GetNearestClusterID(const TSample &sample, tMetric metric)
{
  auto nearest_cluster = this->clusters.end();
  typename TSample::tElement min_distance = std::numeric_limits<typename TSample::tElement>::max();
  for (auto cluster = this->clusters.begin(); cluster != this->clusters.end(); ++cluster)
  {
    typename TSample::tElement distance = metric(cluster->Center(), sample);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_cluster = cluster;
    }
  }
  assert(nearest_cluster != this->clusters.end());
  return std::distance(this->clusters.begin(), nearest_cluster);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
