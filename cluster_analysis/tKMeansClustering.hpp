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
/*!\file    tKMeansClustering.hpp
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

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
#include "rrlib/highgui_wrapper/tWindow.h"
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
//template <typename TSample>
//const typename tKMeansClustering<TSample>::tMetric tKMeansClustering<TSample>::cDEFAULT_METRIC = tKDTree::cDEFAULT_METRIC;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tKMeansClustering constructors
//----------------------------------------------------------------------
template <typename TSample>
template <typename TIterator>
tKMeansClustering<TSample>::tKMeansClustering(unsigned int k,
    TIterator samples_begin, TIterator samples_end,
    typename tKMeansClustering::tMetric metric)
{
  assert(k > 0);
  assert(std::distance(samples_begin, samples_end) > 0);
  this->clusters.reserve(k);
  const tKDTree kd_tree(samples_begin, samples_end);
  this->GenerateInitialClusterPositions(kd_tree.Root(), k);
  assert(this->clusters.size() == k);
  this->Solve(samples_begin, samples_end, kd_tree, metric);
}

template <typename TSample>
template <typename TIterator>
tKMeansClustering<TSample>::tKMeansClustering(unsigned int k,
    TIterator samples_begin, TIterator samples_end,
    const tKDTree &kd_tree,
    typename tKMeansClustering::tMetric metric)
{
  assert(k > 0);
  assert(std::distance(samples_begin, samples_end) > 0);
  this->clusters.reserve(k);
  this->GenerateInitialClusterPositions(kd_tree.Root(), k);
  assert(this->Clusters().size() == k);
  this->Solve(samples_begin, samples_end, kd_tree, metric);
}

template <typename TSample>
template <typename TIterator>
tKMeansClustering<TSample>::tKMeansClustering(TIterator samples_begin, TIterator samples_end,
    TIterator initial_positions_begin, TIterator initial_positions_end,
    typename tKMeansClustering::tMetric metric)
{
  unsigned int k = std::distance(initial_positions_begin, initial_positions_end);
  assert(k > 0);
  assert(std::distance(samples_begin, samples_end) > 0);
  this->clusters.reserve(k);
  const tKDTree kd_tree(samples_begin, samples_end);
  for (auto it = initial_positions_begin; it != initial_positions_end; ++it)
  {
    this->clusters.push_back(*it);
  }
  assert(this->Clusters().size() == k);
  this->Solve(samples_begin, samples_end, kd_tree, metric);
}

template <typename TSample>
template <typename TIterator>
tKMeansClustering<TSample>::tKMeansClustering(TIterator samples_begin, TIterator samples_end,
    TIterator initial_positions_begin, TIterator initial_positions_end,
    const tKDTree &kd_tree,
    typename tKMeansClustering::tMetric metric)
{
  unsigned int k = std::distance(initial_positions_begin, initial_positions_end);
  assert(k > 0);
  assert(std::distance(samples_begin, samples_end) > 0);
  this->clusters.reserve(k);
  for (auto it = initial_positions_begin; it != initial_positions_end; ++it)
  {
    this->clusters.push_back(*it);
  }
  assert(this->Clusters().size() == k);
  this->Solve(samples_begin, samples_end, kd_tree, metric);
}

//----------------------------------------------------------------------
// tKMeansClustering DistanceToNode
//----------------------------------------------------------------------
template <typename TSample>
typename TSample::tElement tKMeansClustering<TSample>::DistanceToNode(const TSample &x, const typename tKDTree::tNode &node, typename tKMeansClustering::tMetric metric) const
{
  // clip a copy of x to the hyper-rectangle ...
  TSample y(x);
  for (size_t i = 0; i < TSample::cDIMENSION; ++i)
  {
    y[i] = std::min(std::max(y[i], node.BoundingBox().Min()[i]), node.BoundingBox().Max()[i]);
  }
  // ... and return the distance
  return metric(x, y);
}

//----------------------------------------------------------------------
// tKMeansClustering UpdateFromKDTreeNode
//----------------------------------------------------------------------
template <typename TSample>
void tKMeansClustering<TSample>::UpdateFromKDTreeNode(const typename tKDTree::tNode &node, typename tKMeansClustering::tMetric metric)
{
  // special treatment for leaves
  if (node.IsLeaf())
  {
    this->clusters[this->GetNearestClusterID(node.CenterOfMass(), metric)].Update(node.CenterOfMass());
    return;
  }

  // find owner candidate for this node
  auto owner_candidate = this->clusters.end();
  typename TSample::tElement shortest_distance = std::numeric_limits<typename TSample::tElement>::max();
  for (auto it = this->clusters.begin(); it != this->clusters.end(); ++it)
  {
    typename TSample::tElement distance = this->DistanceToNode(it->Center(), node, metric);
    if (distance == shortest_distance)
    {
      owner_candidate = this->clusters.end();
    }
    if (distance < shortest_distance)
    {
      shortest_distance = distance;
      owner_candidate = it;
    }
  }

  if (owner_candidate != this->clusters.end())
  {
    // check if this candidate dominates all other clusters
    bool dominating = true;
    for (auto it = this->clusters.begin(); it != this->clusters.end(); ++it)
    {
      if (it != owner_candidate) // no check against the candidate itself
      {
        // get the outermost point of the hyper-rectangle of the node in direction from candidate to challenging centroid
        TSample check_point;
        for (size_t i = 0; i < TSample::cDIMENSION; i++)
        {
          check_point[i] = it->Center()[i] > owner_candidate->Center()[i] ? node.BoundingBox().Max()[i] : node.BoundingBox().Min()[i];
        }

        if (metric(it->Center(), check_point) <= metric(owner_candidate->Center(), check_point))
        {
          dominating = false;
          break;
        }
      }
    }

    if (dominating)
    {
#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
      highgui::tWindow &debug_window(highgui::tWindow::GetInstance("Debug k-means"));
      debug_window.SetColor(std::distance(this->clusters.begin(), owner_candidate));
      debug_window.DrawRectangleShifted(node.BoundingBox().Min().X(), node.BoundingBox().Min().Y(), node.BoundingBox().Max().X(), node.BoundingBox().Max().Y());
#endif

      owner_candidate->Update(node.CenterOfMass(), node.NumberOfPoints());
      return;
    }
  }

  // recursively do the same with the children
  this->UpdateFromKDTreeNode(node.LeftChild(), metric);
  this->UpdateFromKDTreeNode(node.RightChild(), metric);
}

//----------------------------------------------------------------------
// tKMeansClustering Solve
//----------------------------------------------------------------------
template <typename TSample>
template <class TIterator>
void tKMeansClustering<TSample>::Solve(TIterator samples_begin, TIterator samples_end, const tKDTree &kd_tree, const typename tKMeansClustering::tMetric &metric)
{

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
  static_assert(TSample::cDIMENSION == 2, "Debugging of k-means is only supported for 2D samples");
  geometry::tBoundingBox<TSample::cDIMENSION, typename TSample::tElement> bounding_box(samples_begin, samples_end);
  highgui::tWindow &debug_window(highgui::tWindow::GetInstance("Debug k-means",
                                 bounding_box.Max().X() - bounding_box.Min().X() + 1, bounding_box.Max().Y() - bounding_box.Min().Y() + 1,
                                 bounding_box.Min().X(), bounding_box.Min().Y()));
#endif

  bool done = false;
  while (!done)
  {
#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
    debug_window.Clear();
#endif

    // start recursive update of clusters from the kd-tree
    this->UpdateFromKDTreeNode(kd_tree.Root(), metric);

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
    for (size_t i = 0; i < this->clusters.size(); ++i)
    {
      debug_window.SetColor(i);
      for (auto measurement = samples_begin; measurement != samples_end; ++measurement)
      {
        if (this->GetNearestClusterID(*measurement, metric) == i)
        {
          debug_window.DrawPointShifted(measurement->X(), measurement->Y());
        }
      }
      debug_window.DrawCircleShifted(this->clusters[i].Center().X(), this->clusters[i].Center().Y(), 5, true);
    }
    debug_window.SetColor(0);
#endif

    // update all clusters and check if termination is reached
    bool any_update_noticable = false;
    for (auto it = this->clusters.begin(); it != this->clusters.end(); ++it)
    {

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
      TSample old_center = it->Center();
#endif

      bool update_noticable = it->ApplyUpdates(metric);

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
      debug_window.DrawLineShifted(old_center.X(), old_center.Y(), it->Center().X(), it->Center().Y());
#endif

      any_update_noticable = any_update_noticable || update_noticable;
    }
    done = !any_update_noticable;

#ifdef RRLIB_MODEL_FITTING_DEBUG_KMEANS
    debug_window.Render();
#endif

  }

  // post-process resulting structure
  for (auto sample = samples_begin; sample != samples_end; ++sample)
  {
    this->clusters[this->GetNearestClusterID(*sample, metric)].AddSample(*sample);
  }
  for (auto it = this->clusters.begin(); it != this->clusters.end(); ++it)
  {
    it->ComputeSumOfNorms(metric);
  }
}

//----------------------------------------------------------------------
// tKMeansClustering GenerateInitialClusterPositions
//----------------------------------------------------------------------
template <typename TSample>
void tKMeansClustering<TSample>::GenerateInitialClusterPositions(const typename tKDTree::tNode &node, size_t n)
{
  if (n == 0)
  {
    return;
  }

  if (node.IsLeaf())
  {
    n = std::min(n, node.NumberOfPoints());
    // FIXME: Get random subset of size n from the node's points
    for (size_t i = 0; i < n; i++)
    {
      this->clusters.push_back(typename tKMeansClustering::tCluster(node.CenterOfMass()));
    }
  }
  else
  {
    size_t left_n = static_cast<size_t>(std::round(n * node.LeftChild().NumberOfPoints() / static_cast<double>(node.NumberOfPoints())));
    size_t right_n = n - left_n;

    this->GenerateInitialClusterPositions(node.LeftChild(), left_n);
    this->GenerateInitialClusterPositions(node.RightChild(), right_n);
  }
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
