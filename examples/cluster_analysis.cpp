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
/*!\file    rrlib/model_fitting/examples/cluster_analysis.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-01-08
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>
#include <vector>

#include <random>
#include <functional>

#include "rrlib/highgui_wrapper/tWindow.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"

//#define RRLIB_MODEL_FITTING_DEBUG_KMEANS
#include "rrlib/model_fitting/cluster_analysis/tKMeansClustering.h"

//#define RRLIB_MODEL_FITTING_DEBUG_XMEANS
#include "rrlib/model_fitting/cluster_analysis/tXMeansClustering.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::highgui;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef double tElement;
typedef rrlib::geometry::tPoint<2, tElement> tPoint;
typedef rrlib::model_fitting::tClustering<tPoint> tClustering;
typedef rrlib::model_fitting::tKMeansClustering<tPoint> tKMeansClustering;
typedef rrlib::model_fitting::tXMeansClustering<tPoint> tXMeansClustering;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const unsigned int cWINDOW_SIZE  = 600;
const double cNORMAL_DISTRIBUTION_QUANTILE_95_PERCENT = 1.6449;
const double cNORMAL_DISTRIBUTION_QUANTILE_99_PERCENT = 2.3263;
const double cNORMAL_DISTRIBUTION_QUANTILE_99_9_PERCENT = 3.0902;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void DrawPoint(tWindow &window, const tPoint &point)
{
  window.DrawCircleShifted(point.X(), point.Y(), 1, true);
}

void DrawCircle(tWindow &window, const tPoint &center, tElement radius)
{
  window.DrawCircleShifted(center.X(), center.Y(), radius, false);
}

void DrawPoints(tWindow &window, const std::vector<tPoint> &points)
{
  for (auto it = points.begin(); it != points.end(); ++it)
  {
    DrawPoint(window, *it);
  }
}

void DrawClustering(tWindow &window, const tClustering &clustering)
{
  for (size_t i = 0; i < clustering.Clusters().size(); ++i)
  {
    window.SetColor(i);
    DrawPoints(window, clustering.Clusters()[i].Samples());
    window.DrawCircleShifted(clustering.Clusters()[i].Center().X(), clustering.Clusters()[i].Center().Y(), 5, true);
  }
}

void GenerateRandomClusteredPoints(std::vector<tPoint> &points, unsigned int &number_of_clusters,
                                   unsigned int min_number_of_samples, unsigned int max_number_of_samples,
                                   unsigned int min_number_of_clusters, unsigned int max_number_of_clusters,
                                   unsigned int min_cluster_radius, unsigned int max_cluster_radius,
                                   double normal_distribution_quantile_factor, unsigned long seed = time(NULL))
{
  points.clear();

  std::mt19937 rng_engine(seed);

  number_of_clusters = std::uniform_int_distribution<unsigned int>(min_number_of_clusters, max_number_of_clusters)(rng_engine);

  struct tCluster
  {
    tPoint center;
    tElement radius;
    std::normal_distribution<tElement> distribution_x;
    std::normal_distribution<tElement> distribution_y;
  };
  std::vector<tCluster> clusters;

  auto random_cluster_center_component = std::bind(std::uniform_real_distribution<tElement>(-0.5 * cWINDOW_SIZE + max_cluster_radius, 0.5 * cWINDOW_SIZE - max_cluster_radius), rng_engine);
  auto random_cluster_radius = std::bind(std::uniform_real_distribution<tElement>(min_cluster_radius, max_cluster_radius), rng_engine);
  unsigned generated_clusters = 0;
  while (clusters.size() < number_of_clusters && generated_clusters++ < max_number_of_clusters * 10)
  {
    tPoint center(random_cluster_center_component(), random_cluster_center_component());
    tElement radius = random_cluster_radius();
    bool overlapping = false;
    for (auto it = clusters.begin(); it != clusters.end(); ++it)
    {
      if ((it->center - center).Length() < 2 * (it->radius + radius))
      {
        overlapping = true;
        break;
      }
    }
    if (!overlapping)
    {
      clusters.push_back(tCluster());
      clusters.back().center = center;
      clusters.back().radius = radius;
    }
  }

  assert(clusters.size() <= number_of_clusters);
  if (clusters.size() < number_of_clusters)
  {
    number_of_clusters = clusters.size();
    std::cout << "Could not fit more than " << number_of_clusters << " clusters into given area." << std::endl;
  }

  for (size_t i = 0; i < number_of_clusters; ++i)
  {
    clusters[i].distribution_x = std::normal_distribution<tElement>(clusters[i].center.X(), clusters[i].radius / normal_distribution_quantile_factor);
    clusters[i].distribution_y = std::normal_distribution<tElement>(clusters[i].center.Y(), clusters[i].radius / normal_distribution_quantile_factor);
  }

  unsigned int number_of_samples = std::uniform_int_distribution<unsigned int>(min_number_of_samples, max_number_of_samples)(rng_engine);

  rrlib::geometry::tBoundingBox<2, tElement> bounding_box;
  bounding_box.Add(0.5 * tPoint(cWINDOW_SIZE, cWINDOW_SIZE));
  bounding_box.Add(-0.5 * tPoint(cWINDOW_SIZE, cWINDOW_SIZE));

  while (points.size() < number_of_samples)
  {
    size_t cluster_id = std::uniform_int_distribution<size_t>(0, number_of_clusters - 1)(rng_engine);
    tPoint sample(clusters[cluster_id].distribution_x(rng_engine), clusters[cluster_id].distribution_y(rng_engine));
    if (bounding_box.Contains(sample))
    {
      points.push_back(sample);
    }
  }
}

int main(int argc, char **argv)
{
  tWindow &window(tWindow::GetInstance("Test Cluster Analysis", cWINDOW_SIZE, cWINDOW_SIZE, -0.5 * cWINDOW_SIZE, -0.5 * cWINDOW_SIZE));

  std::vector<tPoint> points;
  unsigned int number_of_clusters;

  GenerateRandomClusteredPoints(points, number_of_clusters, 2000, 4000, 3, 15, 20, 60, cNORMAL_DISTRIBUTION_QUANTILE_95_PERCENT);

  window.Clear();
  DrawPoints(window, points);
  window.Render();

  tKMeansClustering k_means_clustering(number_of_clusters, points.begin(), points.end());

  window.Clear();
  DrawPoints(window, points);
  DrawClustering(window, k_means_clustering);
  window.Render();

  tXMeansClustering x_means_clustering(2 * number_of_clusters, points.begin(), points.end());

  window.Clear();
  DrawPoints(window, points);
  DrawClustering(window, x_means_clustering);
  window.Render();

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
