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
/*!\file    tRansacModel.h
 *
 * \author  Tim Braun
 * \author  Tobias Foehst
 *
 * \date    2007-11-16
 *
 * \brief   Contains tRansacModel
 *
 * \b tRansacModel
 *
 * A few words for tRansacModel
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__tRansacModel_h__
#define __rrlib__model_fitting__tRansacModel_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

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
//! Short description of tRansacModel
/*! A general base class that implements the RANSAC algorithm for an
 *  arbitrary model. The model specific details need to be implemented in
 *  a derived class.
 */
template <typename TSample>
class tRansacModel
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TSample tSample;

  explicit tRansacModel(bool local_optimization = false);

  virtual ~tRansacModel() = 0;

  void Initialize(unsigned int expected_number_of_samples);

  void Clear();

  inline void AddSample(const tSample &sample)
  {
    this->samples.push_back(sample);
  }

  template <typename TIterator>
  inline void AddSamples(TIterator begin, TIterator end)
  {
    this->samples.reserve(std::distance(begin, end));
    for (TIterator it = begin; it != end; ++it)
    {
      this->samples.push_back(*it);
    }
  }

  inline void SetLocalOptimization(bool enabled)
  {
    this->local_optimization = enabled;
  }

  const bool DoRANSAC(unsigned int max_iterations, double satisfactory_inlier_ratio = 1.0, double max_error = 1E-6);

  inline const std::vector<tSample> &Samples() const
  {
    return this->samples;
  }

  inline const std::vector<bool> &Assignments() const
  {
    return this->assignments;
  }

  inline const double NumberOfInliers() const
  {
    return this->number_of_inliers;
  }

  inline const double InlierRatio() const
  {
    return this->inlier_ratio;
  }

  inline const double Error() const
  {
    return this->error;
  }

  virtual const size_t MinimalSetSize() const = 0;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  bool local_optimization;
  std::vector<tSample> samples;
  std::vector<bool> assignments;
  size_t number_of_inliers;
  double inlier_ratio;
  double error;

  virtual const char *GetLogDescription() const
  {
    return "tRansacModel";
  }

  void GenerateRandomIndexSet(std::vector<size_t> &index_set, size_t set_size, size_t max_index) const;
  double DetermineConsensusIndexSet(std::vector<size_t> &consensus_index_set, double max_error) const;

  virtual const bool FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set) = 0;
  virtual const bool FitToSampleIndexSet(const std::vector<size_t> &sample_index_set) = 0;
  virtual const double GetSampleError(const tSample &sample) const = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tRansacModel.hpp"

#endif
