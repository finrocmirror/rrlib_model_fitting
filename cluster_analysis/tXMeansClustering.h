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
/*!\file    tXMeansClustering.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 * \brief   Contains tXMeansClustering
 *
 * \b tXMeansClustering
 *
 * The x-means clustering algorithm (D. Pelleg and A. Moore, 2000)
 * determines the number and position of clusters of coordinates in a
 * n-dimensional search space. In that way it extends the classical
 * k-means algorithm, which required a known number of clusters and
 * good initial positions of their centroids. It requires a fast
 * implementation of k-means and starts with an initial assumption that
 * it will find only one cluster. Then it generates recursively for
 * every cluster two new centroids and estimates when the splitting
 * stops yielding a better clustering.
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__cluster_analysis__tXMeansClustering_h__
#define __rrlib__model_fitting__cluster_analysis__tXMeansClustering_h__

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
//! The x-means algorithm (D. Pelleg and A. Moore, 2000)
/*!
 * This class provides a generic implementation of the x-means clustering
 * algorithm. Therefore, it is a template that takes the following
 * parameters:
 *
 * \param Tmeasurement_dimension   The dimension of the search space (and the measurements)
 * \param TMeasurementContent      The basic data type of the measurements (e.g. float or double)
 *
 * \b Usage
 * \code
 * #include <cstdlib>
 * #include <iostream>
 * #include <iterator>
 * #include <vector>
 * #include "general/tXMeansClustering.h"
 *
 * typedef tXMeansClustering<2, float> tClustering;  // just use a typedef to instantiate the template
 *
 * // an own metric to show the generic capabilities of this class
 * // for simplicity it is recommended to use the provided typedef of the template-class, bearing in mind that tMeasurement is always an instantiation of rrlib::math::tVector
 * struct tManhattanNorm : public tClustering::tMetric
 * {
 *   inline const tClustering::tMetric::result_type operator() (const tClustering::tMetric::first_argument_type &x, const tClustering::tMetric::second_argument_type &y) const {
 *     tClustering::tMetric::result_type result = 0;
 *     for (size_t i = 0; i < tClustering::tMeasurement::eDIMENSION; i++)
 *     {
 *       result += std::abs(x[i] - y[i]);
 *     }
 *     return result;
 *   }
 * };
 *
 * int main(int argc, char **argv)
 * {
 *   std::vector<tClustering::tMeasurement> data;
 *   // ... fill data
 *
 *   tClustering clustering(data.size(), data);                   // using the default (Euklidian) norm
 * //  tClustering clustering(data.size(), data, tManhattanNorm()); // using an own defined norm
 *
 *   std::cout << "Found " << clustering.GetNumberOfClusters() << " clusters:" << std::endl;
 *   for (size_t i = 0; i < clustering.GetNumberOfClusters(); i++)
 *   {
 *     std::cout << std::endl << "cluster " << i << " at " << clustering[i].Position() << ":" << std::endl;
 *     const std::vector<tClustering::tMeasurement> &measurements(clustering[i].GetMeasurements());
 *     std::copy(measurements.begin(), measurements.end(), std::ostream_iterator<tClustering::tMeasurement>(std::cout, " "));
 *     std::cout << std::endl;
 *   }
 *
 *   return EXIT_SUCCESS;
 * }
 *
 * \endcode
 */
template <typename TSample>
class tXMeansClustering : public tClustering<TSample>
{
  /*!
   * \brief An instantiation of tKMeansClustering that is used by this algorithm
   */
  typedef model_fitting::tKMeansClustering<TSample> tKMeansClustering;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \brief The ctor of tXMeansClustering
   *
   * \param max_cluster    The upper bound for the number of clusters to arrange
   * \param measurements   The measurements to process
   * \param epsilon        The termination-criterion
   */
  template <typename TIterator>
  tXMeansClustering(unsigned int max_clusters,
                    TIterator samples_begin, TIterator samples_end,
                    typename tXMeansClustering::tMetric metric = TSample::cEUCLIDEAN_DISTANCE);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*!
   * \brief The internal data structure storing the clusters
   */
  class tClusterCandidate
  {

    //----------------------------------------------------------------------
    // Public methods and typedefs
    //----------------------------------------------------------------------
  public:

    tClusterCandidate(const typename tXMeansClustering::tCluster &cluster) : cluster(cluster), bvalue(0) {}

    inline typename tXMeansClustering::tCluster &Cluster()
    {
      return this->cluster;
    }

    inline const std::vector<typename tXMeansClustering::tCluster> &Children() const
    {
      return this->children;
    }

    inline double BValue() const
    {
      return this->bvalue;
    }

    inline void Split(typename tXMeansClustering::tMetric metric);

    //----------------------------------------------------------------------
    // Private fields and methods
    //----------------------------------------------------------------------
  private:

    typename tXMeansClustering::tCluster cluster;

    std::vector<typename tXMeansClustering::tCluster> children;

    double bvalue;

    double ComputeBIC(const std::vector<typename tXMeansClustering::tCluster> &clusters) const;
  };

  /*!
   * \brief Execute the algorithm
   *
   * \param max_clusters                An upper limit for the numer of found clusters as termination criterion
   * \param measurements                The measurements to process
   * \param metric                      A functor which computes an appropriate metric
   * \param epsilon                     The termination-criterion
   */
  template <typename TIterator>
  void Solve(unsigned int max_clusters, TIterator samples_begin, TIterator samples_end, typename tXMeansClustering::tMetric metric);

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/model_fitting/cluster_analysis/tXMeansClustering.hpp"

#endif
