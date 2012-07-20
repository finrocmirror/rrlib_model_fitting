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
    unsigned int max_iterations, double satisfactory_support_ratio, double max_error,
    bool local_optimization)
  : tRansacModel(local_optimization)
{
  this->Initialize(std::distance(begin, end));
  for (TIterator it = begin; it != end; ++it)
  {
    this->AddSample(*it);
  }
  if (!this->DoRANSAC(max_iterations, satisfactory_support_ratio, max_error))
  {
    throw std::runtime_error("Failed to fit RANSAC model during construction!");
  }
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial FitToMinimalSampleIndexSet
//----------------------------------------------------------------------
template <size_t Tdegree>
const bool tRansacLeastSquaresPolynomial<Tdegree>::FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  // ensure distinct points
  for (std::vector<size_t>::const_iterator it = sample_index_set.begin(); it != sample_index_set.end(); ++it)
  {
    std::vector<size_t>::const_iterator kt = it;
    for (++kt; kt != sample_index_set.end(); ++kt)
    {
      if (IsEqual(this->Samples()[*it], this->Samples()[*kt]))
      {
        return false;
      }
    }
  }
  return this->FitToSampleIndexSet(sample_index_set);
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial FitToSampleIndexSet
//----------------------------------------------------------------------
template <size_t Tdegree>
const bool tRansacLeastSquaresPolynomial<Tdegree>::FitToSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  std::vector<tSample> chosen_samples;
  chosen_samples.reserve(sample_index_set.size());
  for (typename std::vector<size_t>::const_iterator it = sample_index_set.begin(); it != sample_index_set.end(); ++it)
  {
    chosen_samples.push_back(this->Samples()[*it]);
  }
  try
  {
    this->UpdateModelFromSampleSet(chosen_samples.begin(), chosen_samples.end());
  }
  catch (std::logic_error &exception)
  {
    RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Failed to update model from sample set: ", exception.what());
    return false;
  }
  return true;
}

//----------------------------------------------------------------------
// tRansacLeastSquaresPolynomial GetSampleError
//----------------------------------------------------------------------
template <size_t Tdegree>
const double tRansacLeastSquaresPolynomial<Tdegree>::GetSampleError(const tSample &sample) const
{
  return AbsoluteValue(sample.Y() - (*this)(sample.X()));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
