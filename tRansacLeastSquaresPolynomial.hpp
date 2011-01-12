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
/*!\file    tRansacLeastSquaresPolynomial.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-30
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

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
// tRansacLeastSquaresPolynomial constructors
//----------------------------------------------------------------------
template <size_t Tdegree>
tRansacLeastSquaresPolynomial<Tdegree>::tRansacLeastSquaresPolynomial(bool local_optimization)
    : tRansacModel(local_optimization)
{}

template <size_t Tdegree>
template <typename TIterator>
tRansacLeastSquaresPolynomial<Tdegree>::tRansacLeastSquaresPolynomial(TIterator begin, TIterator end,
    unsigned int max_iterations, float satisfactory_support_ratio, float max_error,
    bool local_optimization)
    : tRansacModel(local_optimization)
{
  this->Init(std::distance(begin, end));
  for (TIterator it = begin; it != end; ++it)
  {
    this->AddSample(*it);
  }
  this->DoRANSAC(max_iterations, satisfactory_support_ratio, max_error);
}

template <size_t Tdegree>
template <typename TSTLContainer>
tRansacLeastSquaresPolynomial<Tdegree>::tRansacLeastSquaresPolynomial(const TSTLContainer &samples,
    unsigned int max_iterations, float satisfactory_support_ratio, float max_error,
    bool local_optimization)
    : tRansacModel(local_optimization)
{
  this->Init(samples.size());
  for (typename TSTLContainer::const_iterator it = samples.begin(); it != samples.end(); ++it)
  {
    this->AddSample(*it);
  }
  this->DoRANSAC(max_iterations, satisfactory_support_ratio, max_error);
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial FitToMinimalSampleSet
//----------------------------------------------------------------------
template <size_t Tdegree>
const bool tRansacLeastSquaresPolynomial<Tdegree>::FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  return this->FitToSampleIndexSet(sample_index_set);
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial FitToSampleSet
//----------------------------------------------------------------------
template <size_t Tdegree>
const bool tRansacLeastSquaresPolynomial<Tdegree>::FitToSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  std::vector<tSample> chosen_samples;
  chosen_samples.reserve(sample_index_set.size());
  for (typename std::vector<size_t>::const_iterator it = sample_index_set.begin(); it != sample_index_set.end(); ++it)
  {
    chosen_samples.push_back(this->GetSamples()[*it]);
  }
  this->UpdateModelFromSampleSet(chosen_samples);
  return true;
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial GetSampleError
//----------------------------------------------------------------------
template <size_t Tdegree>
const float tRansacLeastSquaresPolynomial<Tdegree>::GetSampleError(const tSample &sample) const
{
  return std::abs(sample.Y() - (*this)(sample.X()));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
