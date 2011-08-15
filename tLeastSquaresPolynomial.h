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
/*!\file    tLeastSquaresPolynomial.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-30
 *
 * \brief   Contains tLeastSquaresPolynomial
 *
 * \b tLeastSquaresPolynomial
 *
 * A few words for tLeastSquaresPolynomial
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__tLeastSquaresPolynomial_h__
#define __rrlib__model_fitting__tLeastSquaresPolynomial_h__

#include "rrlib/math/tPolynomial.h"
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

#include "rrlib/math/tVector.h"

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
//! Short description of tLeastSquaresPolynomial
/*! A more detailed description of tLeastSquaresPolynomial, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdegree>
class tLeastSquaresPolynomial : public math::tPolynomial<Tdegree>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef math::tVec2d tSample;

  tLeastSquaresPolynomial();

  /*!
   */
  template <typename TIterator>
  tLeastSquaresPolynomial(TIterator begin, TIterator end);

  inline double GetStandardDeviation() const
  {
    return this->sigma;
  }

  template <typename TIterator>
  void UpdateModelFromSampleSet(TIterator begin, TIterator end);

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  template <typename TIterator>
  void DoLinearRegression(TIterator begin, TIterator end);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double sigma;

  virtual const char *GetLogDescription() const
  {
    return "tLeastSquaresPolynomial";
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tLeastSquaresPolynomial.hpp"

#endif
