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
/*!\file    tKMeansClustering.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 * \brief   Contains tKMeansClustering
 *
 * \b tKMeansClustering
 *
 * The k-means clustering algorithm (H. Steinhaus, 1956), which groups
 * an given set of measurements into \e k clusters. This implementation
 * accelerates the classical approach by using a kd-tree and geometric
 * reasoning (D. Pelleg and A. Moore, 1999)
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__cluster_analysis__tKMeansClustering_h__
#define __rrlib__model_fitting__cluster_analysis__tKMeansClustering_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

#include "rrlib/geometry/space_partitioning/tKDTree.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/cluster_analysis/tClustering.h"

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
//! The k-means algorithm
/*!
 * This class provides a generic implementation of the k-means clustering
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
 * #include "general/tKMeansClustering.h"
 *
 * typedef tKMeansClustering<2, float> tClustering;  // just use a typedef to instantiate the template
 *
 * // an own metric to show the generic capabilities of this class
 * // for simplicity it is recommended to use the provided typedef of the template-class, bearing in mind that tMeasurement is always an instantiation of rrlib::math::tVector
 * struct tManhattanNorm : public tClustering::tMetric
 * {
 *   inline const tClustering::tMetric::result_type operator() (const tClustering::tMetric::first_argument_type &x, const tClustering::tMetric::second_argument_type &y) const
 *   {
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
 *   // assuming 6 clusters the algorithm can be executed by
 *   tClustering clustering(6, data);                   // using the default (Euklidian) norm
 * //  tClustering clustering(6, data, tManhattanNorm()); // using an own defined norm
 *
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
 * \endcode
 */
template <typename TSample>
class tKMeansClustering : public tClustering<TSample>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \brief An instantiation of the tKDTree template that is used in this algorithm
   */
  typedef geometry::tKDTree<TSample::cDIMENSION, typename TSample::tElement> tKDTree;

  /*!
   * \brief The ctor of tKMeansClustering
   *
   * \param k                           The number of clusters to arrange
   * \param measurements                The measurements to process
   * \param kd_tree                     A pre-computed kd-tree on the measurements that can be used
   * \param initial_cluster_positions   A list of initial positions where clusters are expected
   * \param epsilon                     The termination-criterion
   */
  template <typename TIterator>
  tKMeansClustering(unsigned int k,
                    TIterator samples_begin, TIterator samples_end,
                    typename tKMeansClustering::tMetric metric = tKMeansClustering::cDEFAULT_METRIC);

  template <typename TIterator>
  tKMeansClustering(unsigned int k,
                    TIterator samples_begin, TIterator samples_end,
                    const tKDTree &kd_tree,
                    typename tKMeansClustering::tMetric metric = tKMeansClustering::cDEFAULT_METRIC);

  template <typename TIterator>
  tKMeansClustering(TIterator samples_begin, TIterator samples_end,
                    TIterator initial_positions_begin, TIterator initial_positions_end,
                    typename tKMeansClustering::tMetric metric = tKMeansClustering::cDEFAULT_METRIC);

  template <typename TIterator>
  tKMeansClustering(TIterator samples_begin, TIterator samples_end,
                    TIterator initial_positions_begin, TIterator initial_positions_end,
                    const tKDTree &kd_tree,
                    typename tKMeansClustering::tMetric metric = tKMeansClustering::cDEFAULT_METRIC);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*!
   * \brief Get the distance between a point and the hyper-rectangle of a kd-tree-node
   *
   * \param x        The point
   * \param node     The node
   * \param metric   The functor which computes an appropriate metric
   *
   * \return The distance from the point to the node
   */
  typename TSample::tElement DistanceToNode(const TSample &x, const typename tKDTree::tNode &node, typename tKMeansClustering::tMetric metric) const;

  /*!
   * \brief Update the clusters from the given kd-tree node recursively
   *
   * \param node     The kd-tree node to process
   * \param metric   The functor which computes an appropriate metric
   */
  void UpdateFromKDTreeNode(const typename tKDTree::tNode &node, typename tKMeansClustering::tMetric metric);

  /*!
   * \brief Execute the algorithm
   *
   * \param measurements                The measurements to process
   * \param kd_tree                     A precomputed kd-tree on the measurements that can be used
   * \param metric                      A functor which computes an appropriate metric
   * \param epsilon                     The termination-criterion
   */
  template <typename TIterator>
  void Solve(TIterator samples_begin, TIterator samples_end, const tKDTree &kd_tree, const typename tKMeansClustering::tMetric &metric);

  /*!
   * \brief Generates cluster positions using a heuristic on the kd-tree
   *
   * \param node   The root of the subtree that should be used to generate \a n initial cluster positions
   * \param n      The number of initial cluster positions that should be generated
   */
  void GenerateInitialClusterPositions(const typename tKDTree::tNode &node, size_t n);

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/model_fitting/cluster_analysis/tKMeansClustering.hpp"

#endif
