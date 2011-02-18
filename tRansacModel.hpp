//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    tRansacModel.hpp
 *
 * \author  Tim Braun
 * \author  Tobias Foehst
 *
 * \date    2007-11-16
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <algorithm>

#include "rrlib/util/stl_container/join.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/definitions.h"

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
// tRansacModel constructors
//----------------------------------------------------------------------
template <typename TSample>
tRansacModel<TSample>::tRansacModel(bool local_optimization)
    : local_optimization(local_optimization),
    inlier_ratio(0),
    error(0)
{}

//----------------------------------------------------------------------
// tRansacModel destructor
//----------------------------------------------------------------------
template <typename TSample>
tRansacModel<TSample>::~tRansacModel()
{}

//----------------------------------------------------------------------
// tRansacModel Initialize
//----------------------------------------------------------------------
template <typename TSample>
void tRansacModel<TSample>::Initialize(unsigned int expected_number_of_samples)
{
  this->Clear();
  this->samples.reserve(expected_number_of_samples);
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Initialized for ", expected_number_of_samples, " samples.");
}

//----------------------------------------------------------------------
// tRansacModel Clear
//----------------------------------------------------------------------
template <typename TSample>
void tRansacModel<TSample>::Clear()
{
  this->samples.clear();
  this->assignments.clear();
  this->inlier_ratio = 0;
  this->error = 0;
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Model cleared.");
}

//----------------------------------------------------------------------
// tRansacModel DoRANSAC
//----------------------------------------------------------------------
template <typename TSample>
const bool tRansacModel<TSample>::DoRANSAC(unsigned int max_iterations, double satisfactory_inlier_ratio, double max_error)
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Performing RANSAC algorithm.");

  if (this->samples.size() < this->MinimalSetSize())
  {
    RRLIB_LOG_STREAM(logging::eLL_ERROR, "At least ", this->MinimalSetSize(), " samples must be added to construct model!");
    this->Clear();
    return false;
  }

  std::vector<size_t> minimal_index_set;
  minimal_index_set.reserve(this->MinimalSetSize());

  std::vector<size_t> consensus_index_set;
  consensus_index_set.reserve(this->samples.size());

  std::vector<size_t> best_consensus_index_set;
  best_consensus_index_set.reserve(this->samples.size());

  size_t satisfactory_support = std::round(satisfactory_inlier_ratio * this->samples.size());
  size_t max_support = 0;
  double min_error = std::numeric_limits<double>::max();

  // main RANSAC loop
  for (unsigned int iteration = 0; iteration < max_iterations; ++iteration)
  {
    RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Iteration: ", iteration);

    // generate indices for minimal random subset of all samples
    this->GenerateRandomIndexSet(minimal_index_set, this->MinimalSetSize(), this->samples.size() - 1);

    RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_3, "Random subset: ", util::Join(minimal_index_set, ", "));

    // fit model to minimal sample set
    if (!this->FitToMinimalSampleIndexSet(minimal_index_set))
    {
      RRLIB_LOG_STREAM(logging::eLL_DEBUG_WARNING, "Failed to construct model from minimal sample set. Skipping iteration.");
      continue;
    }

    double total_error = this->DetermineConsensusIndexSet(consensus_index_set, max_error);
    size_t support = consensus_index_set.size();

    // proceed if we found better support or lower error
    if (support > max_support || (support == max_support && total_error < min_error))
    {
      RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Found better model with support ", support, " and total inlier error ", total_error);

      max_support = support;
      min_error = total_error;
      best_consensus_index_set = consensus_index_set;

      if (this->local_optimization)
      {
        if (!this->FitToSampleIndexSet(best_consensus_index_set))
        {
          RRLIB_LOG_STREAM(logging::eLL_DEBUG_WARNING, "Failed to optimize model locally. Continuing with unoptimized model.\n");
        }
        else
        {
          double total_error = this->DetermineConsensusIndexSet(consensus_index_set, max_error);
          size_t support = consensus_index_set.size();

          if (support > max_support || (support == max_support && total_error < min_error))
          {
            RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Local Optimization yielded better model with support ", support, " and total inlier error ", total_error);

            max_support = support;
            min_error = total_error;
            best_consensus_index_set = consensus_index_set;
          }
        }
      }

      // break if support requirements are already met
      if (max_support >= satisfactory_support)
      {
        RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Reached satisfactory support ratio. Stopping iteration.");
        break;
      }
    }

  }

  // see if we found a model
  if (max_support == 0)
  {
    RRLIB_LOG_STREAM(logging::eLL_ERROR, "Failed to find a consensus set. Could not construct model.");
    this->Clear();
    return false;
  }

  if (!this->FitToSampleIndexSet(best_consensus_index_set))
  {
    RRLIB_LOG_STREAM(logging::eLL_ERROR, "Failed to construct model from largest consensus set. Could not construct model.");
    this->Clear();
    return false;
  }

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Final model has been constructed from largest consensus set (size ", best_consensus_index_set.size(), " / ", this->samples.size(), ").");

  this->assignments.resize(this->samples.size(), false);
  for (typename std::vector<size_t>::const_iterator it = best_consensus_index_set.begin(); it != best_consensus_index_set.end(); ++it)
  {
    this->assignments[*it] = true;
  }

  this->number_of_inliers = max_support;
  this->inlier_ratio = static_cast<double>(max_support) / this->samples.size();
  this->error = min_error;

  return true;
}

//----------------------------------------------------------------------
// tRansacModel GenerateRandomIndexSubset
//----------------------------------------------------------------------
template <typename TSample>
void tRansacModel<TSample>::GenerateRandomIndexSet(std::vector<size_t> &index_set, size_t set_size, size_t max_index) const
{
  index_set.clear();
  index_set.reserve(set_size);
  for (size_t i = 0; i < set_size; ++i)
  {
    size_t index;
    do
    {
      index = rand() % (max_index + 1);
    }
    while (std::find(index_set.begin(), index_set.end(), index) != index_set.end());
    index_set.push_back(index);
  }
}

//----------------------------------------------------------------------
// tRansacModel DetermineConsensusIndexSet
//----------------------------------------------------------------------
template <typename TSample>
double tRansacModel<TSample>::DetermineConsensusIndexSet(std::vector<size_t> &consensus_index_set, double max_error) const
{
  consensus_index_set.clear();
  double total_error = 0.0;
  for (size_t i = 0; i < this->samples.size(); ++i)
  {
    double error = this->GetSampleError(this->samples[i]);
    if (error <= max_error)
    {
      total_error += error;
      consensus_index_set.push_back(i);
    }
  }
  return total_error;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
