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
/*!\file    tLeastSquaresPolynomial.hpp
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
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tLUDecomposition.h"
#include "rrlib/math/tCholeskyDecomposition.h"

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
// tLeastSquaresPolynomial constructors
//----------------------------------------------------------------------
template <size_t Tdegree>
tLeastSquaresPolynomial<Tdegree>::tLeastSquaresPolynomial()
    : sigma(0)
{}

template <size_t Tdegree>
template <typename TIterator>
tLeastSquaresPolynomial<Tdegree>::tLeastSquaresPolynomial(TIterator begin, TIterator end)
    : sigma(0)
{
  this->DoLinearRegression(begin, end);
}

//----------------------------------------------------------------------
// tLeastSquaresPolynomial UpdateModelFromSampleSet
//----------------------------------------------------------------------
template <size_t Tdegree>
template <typename TIterator>
void tLeastSquaresPolynomial<Tdegree>::UpdateModelFromSampleSet(TIterator begin, TIterator end)
{
  this->DoLinearRegression(begin, end);
}

//----------------------------------------------------------------------
// tLeastSquaresPolynomial DoLinearRegression
//----------------------------------------------------------------------
template <size_t Tdegree>
template <typename TIterator>
void tLeastSquaresPolynomial<Tdegree>::DoLinearRegression(TIterator begin, TIterator end)
{
  /*
   * After some derivation work doing linear regression in this case means solving
   *
   * S(xi^0*xi^0)   ...   S(xi^0*xi^n)   |   S(xi^0*yi)
   *      .                    .         |       .
   *      .                    .         |       .
   * S(xi^n*xi^0)   ...   S(xi^n*xi^n)   |   S(xi^n*yi)
   *
   */
  math::tMatrix < Tdegree + 1, Tdegree + 1, double, math::matrix::Symmetrical > A;
  math::tVector < Tdegree + 1, double > b;

  for (TIterator it = begin; it != end; ++it)
  {
    RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Considering sample ", *it);

    double coefficient_components[2 * Tdegree + 1];
    coefficient_components[0] = 1;
    for (size_t i = 1; i < 2 * Tdegree + 1; ++i)
    {
      coefficient_components[i] = it->X() * coefficient_components[i - 1];
    }

    for (size_t row = 0; row < Tdegree + 1; ++row)
    {
      for (size_t column = 0; column <= row; ++column)
      {
        A[row][column] += coefficient_components[row + column];
      }
      b[row] += coefficient_components[row] * it->Y();
    }
  }

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Solving ", A, " x = ", b);

  math::tCholeskyDecomposition < Tdegree + 1 > cholesky_decomposition(A);
  math::tVector < Tdegree + 1 > solution = cholesky_decomposition.Solve(b);

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "x = ", solution);

  for (size_t i = 0; i < Tdegree + 1; ++i)
  {
    this->SetCoefficient(i, solution[i]);
  }

  // calculate standard deviation
  size_t number_of_samples = 0;
  for (TIterator it = begin; it != end; ++it)
  {
    double error = it->Y() - (*this)(it->X());
    this->sigma += error * error;
    number_of_samples++;
  }
  this->sigma = std::sqrt(this->sigma / (number_of_samples - 1));

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "sigma = ", this->sigma);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
