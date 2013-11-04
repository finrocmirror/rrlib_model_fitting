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
/*!\file    tXMeansClustering.hpp
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

#ifdef _LIB_RRLIB_HIGHGUI_WRAPPER_PRESENT_
# ifdef RRLIB_MODEL_FITTING_DEBUG_XMEANS
#  include "rrlib/highgui_wrapper/tWindow.h"
# endif
#else
# undef RRLIB_MODEL_FITTING_DEBUG_XMEANS
#endif

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
// tXMeansClustering constructors
//----------------------------------------------------------------------
template <typename TSample>
template <typename TIterator>
tXMeansClustering<TSample>::tXMeansClustering(unsigned int max_clusters,
    TIterator samples_begin, TIterator samples_end,
    typename tXMeansClustering::tMetric metric)
{
  assert(max_clusters > 0);
  this->Solve(max_clusters, samples_begin, samples_end, metric);
}

//----------------------------------------------------------------------
// tXMeansClustering Solve
//----------------------------------------------------------------------
template <typename TSample>
template <typename TIterator>
void tXMeansClustering<TSample>::Solve(unsigned int max_clusters, TIterator samples_begin, TIterator samples_end, typename tXMeansClustering::tMetric metric)
{
  std::cout << "Preprocessing" << std::endl;

  // scale input data to [0,1] for all axes
  const geometry::tBoundingBox<TSample::cDIMENSION, typename TSample::tElement> bounding_box(samples_begin, samples_end);
  const TSample sample_extension = bounding_box.Max() - bounding_box.Min();

  std::vector<TSample> scaled_samples(samples_begin, samples_end);

  for (auto it = scaled_samples.begin(); it != scaled_samples.end(); ++it)
  {
    *it -= bounding_box.Min();
    for (size_t i = 0; i < TSample::cDIMENSION; ++i)
    {
      (*it)[i] /= sample_extension[i];
    }
  }

  typename tKMeansClustering::tKDTree kd_tree(scaled_samples.begin(), scaled_samples.end());

  // the first clustering: in fact located at the mean of all given samples
  tKMeansClustering initial_clustering(1, scaled_samples.begin(), scaled_samples.end(), kd_tree, metric);
  std::vector<tClusterCandidate> cluster_candidates(initial_clustering.Clusters().begin(), initial_clustering.Clusters().end());
  std::cout << "Initial cluster candidates: ";
  for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
  {
    std::cout << it->Cluster().Center() << " ";
  }
  std::cout << std::endl;

  std::cout << "Done." << std::endl;

#ifdef RRLIB_MODEL_FITTING_DEBUG_XMEANS
  static_assert(TSample::cDIMENSION == 2, "Debugging of x-means is only supported for 2D samples");
  highgui::tWindow &debug_window(highgui::tWindow::GetInstance("Debug x-means", 500, 500));
  for (auto it = scaled_samples.begin(); it != scaled_samples.end(); ++it)
  {
    debug_window.DrawPointNormalized(it->X(), it->Y());
  }
  for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
  {
    debug_window.SetColor(std::distance(cluster_candidates.begin(), it));
    debug_window.DrawCircleNormalized(it->Cluster().Center().X(), it->Cluster().Center().Y(), 0.01, true);
  }
  debug_window.Render();
#endif

  size_t current_number_of_clusters = 0;
  while (true)
  {
    // terminate if number of clusters did not change or reached the given limit
    if (current_number_of_clusters >= cluster_candidates.size() || cluster_candidates.size() == max_clusters)
    {
      break;
    }
    current_number_of_clusters = cluster_candidates.size();

    std::cout << "Split and k-mean tentative clusters..." << std::endl;

    for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
    {
      it->Split(metric);
    }

    std::cout << "Done k-means for tentative clusters" << std::endl;
    std::cout << "Computing cutoff value for new candidates..." << std::endl;

    // update cluster candidates (surviving vs. dead clusters)
    std::sort(cluster_candidates.begin(), cluster_candidates.end(), [](const tClusterCandidate & a, const tClusterCandidate & b)
    {
      return a.BValue() < b.BValue();
    });
    double cutoff = 0.0; // default cutoff

    // reason about number of candidates preferring children
    size_t cutoff_index = std::distance(cluster_candidates.begin(), std::upper_bound(cluster_candidates.begin(), cluster_candidates.end(), cutoff, [](double bvalue, const tClusterCandidate & x)
    {
      return bvalue < x.BValue();
    }));
    cutoff_index = std::min(cutoff_index, cluster_candidates.size() - 1);

    // clip number of cluster candidates preferring children with respect to the maximum number of clusters
    cutoff_index = std::min(cutoff_index, (int)max_clusters - cluster_candidates.size());
    cutoff = cluster_candidates[cutoff_index].BValue();
    if (cluster_candidates.size() == 1)
    {
      cutoff = cluster_candidates[0].BValue() + 1.0;
    }

    std::cout << "cutoff = " << cutoff << ", cutoff_index = " << cutoff_index << std::endl;

    std::vector<tClusterCandidate> new_cluster_candidates;
    new_cluster_candidates.reserve((size_t)(1.5 * cluster_candidates.size()));
    for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
    {

      std::cout << "Cluster " << std::distance(cluster_candidates.begin(), it) << " BVALUE " << it->BValue() << std::endl;

      if (it->BValue() < cutoff)
      {
        // kill parent, add children
        new_cluster_candidates.insert(new_cluster_candidates.end(), it->Children().begin(), it->Children().end());
      }
      else
      {
        // kill children
        new_cluster_candidates.push_back(*it);
      }
    }
    cluster_candidates = new_cluster_candidates;

    std::cout << "K-mean surviving clusters..." << std::endl;

    // correction clustering
    std::vector<TSample> cluster_candidate_positions;
    cluster_candidate_positions.reserve(cluster_candidates.size());
    for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
    {
      cluster_candidate_positions.push_back(it->Cluster().Center());
    }

    tKMeansClustering correction_clustering(scaled_samples.begin(), scaled_samples.end(), cluster_candidate_positions.begin(), cluster_candidate_positions.end(), kd_tree, metric);
    cluster_candidates.clear();
    cluster_candidates.assign(correction_clustering.Clusters().begin(), correction_clustering.Clusters().end());

    std::cout << "Done k-means for surviving clusters" << std::endl;

#ifdef RRLIB_MODEL_FITTING_DEBUG_XMEANS
    std::cout << "These are the surviving clusters..." << std::endl;
    debug_window.Clear();
    for (auto it = scaled_samples.begin(); it != scaled_samples.end(); ++it)
    {
      debug_window.DrawPointNormalized(it->X(), it->Y());
    }
    for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
    {
      debug_window.SetColor(std::distance(cluster_candidates.begin(), it));
      debug_window.DrawCircleNormalized(it->Cluster().Center().X(), it->Cluster().Center().Y(), 0.01, true);
    }
    debug_window.Render();
#endif

  }

  std::cout << "Postprocessing..." << std::endl;

  // post-process resulting data structure
  std::vector<TSample> postprocessing_centers;
  postprocessing_centers.reserve(cluster_candidates.size());
  for (auto it = cluster_candidates.begin(); it != cluster_candidates.end(); ++it)
  {
    postprocessing_centers.push_back(it->Cluster().Center());
    for (size_t i = 0; i < TSample::cDIMENSION; ++i)
    {
      postprocessing_centers.back()[i] *= sample_extension[i];
    }
    postprocessing_centers.back() += bounding_box.Min();
  }
  tKMeansClustering postprocessing_clustering(samples_begin, samples_end, postprocessing_centers.begin(), postprocessing_centers.end(), metric);
  this->clusters.assign(postprocessing_clustering.Clusters().begin(), postprocessing_clustering.Clusters().end());

  std::cout << "Done." << std::endl;

}

//----------------------------------------------------------------------
// tXMeansClustering::tClusterCandidate Split
//----------------------------------------------------------------------
template <typename TSample>
void tXMeansClustering<TSample>::tClusterCandidate::Split(typename tXMeansClustering::tMetric metric)
{
  if (this->cluster.Samples().size() < 2)
  {
    return;
  }

  // split and k-mean
  std::cout << "splitting for " << this->Cluster().Samples().size() << " samples" << std::endl;
  this->children.clear();
  std::vector<TSample> &samples = const_cast<std::vector<TSample> &>(this->Cluster().Samples());
  tKMeansClustering clustering(2, samples.begin(), samples.end(), metric);
  this->children.assign(clustering.Clusters().begin(), clustering.Clusters().end());

  // decide if parent or children are better candidates to represent the samples
  std::vector<typename tXMeansClustering::tCluster> parent;
  parent.push_back(this->cluster);

  this->bvalue = this->ComputeBIC(parent) - this->ComputeBIC(this->children);
}

//----------------------------------------------------------------------
// tXMeansClustering::tClusterCandidate ComputeBIC
//----------------------------------------------------------------------
template <typename TSample>
double tXMeansClustering<TSample>::tClusterCandidate::ComputeBIC(const std::vector<typename tXMeansClustering::tCluster> &clusters) const
{
  std::cout << "ComputeBIC:" << std::endl;

  unsigned int number_of_clusters = clusters.size();
  unsigned int number_of_parameters = ((number_of_clusters - 1)                       // probabilities
                                       + number_of_clusters * TSample::cDIMENSION     // means
                                       + number_of_clusters);                         // variance parameters

  std::cout << "\tNumber of clusters = " << number_of_clusters << std::endl;
  std::cout << "\tNumber of parameters = " << number_of_parameters << std::endl;

  unsigned int total_number_of_measurements = 0;
  typename TSample::tElement total_sum_of_norms = 0;
  for (auto it = clusters.begin(); it != clusters.end(); ++it)
  {
    total_number_of_measurements += it->Samples().size();
    total_sum_of_norms += it->SumOfNorms();

    std::cout << "\t\tLocal number of measurements = " << it->Samples().size() << std::endl;
    std::cout << "\t\tLocal sum of norms = " << it->SumOfNorms() << std::endl;
  }

  double total_variance = total_sum_of_norms / (total_number_of_measurements - number_of_clusters);

  std::cout << "\tTotal number of measurements = " << total_number_of_measurements << std::endl;
  std::cout << "\tTotal sum of norms = " << total_sum_of_norms << std::endl;
  std::cout << "\tTotal variance = " << total_variance << std::endl;

  double log_likelihood = 0;
  for (auto it = clusters.begin(); it != clusters.end(); ++it)
  {
    unsigned int number_of_measurements = it->Samples().size();
    if (number_of_measurements > 1)
    {
      double local_log_likelihood = (number_of_measurements * log(number_of_measurements)
                                     - number_of_measurements * log(total_number_of_measurements)
                                     - number_of_measurements / 2.0 * log(2.0 * M_PI)
                                     - number_of_measurements * TSample::cDIMENSION / 2.0 * log(total_variance)
                                     - it->SumOfNorms() / (2.0 * total_variance));

      std::cout << "\t\tLocal log Likelihood = " << local_log_likelihood << std::endl;

      log_likelihood += local_log_likelihood;
    }
  }

  double bic_score = log_likelihood - number_of_parameters / 2.0 * log(total_number_of_measurements);

  std::cout << "\tLog Likelihood = " << log_likelihood << std::endl;
  printf("\tLog Likelihood = %6.3g\n", log_likelihood);
  printf("\tBIC score = %6.3g\n", bic_score);

  return bic_score;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
