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
/*!\file    tRansacLeastSquaresPolynomial.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-30
 *
 * \brief   Contains tRansacLeastSquaresPolynomial
 *
 * \b tRansacLeastSquaresPolynomial
 *
 * A few words for tRansacLeastSquaresPolynomial
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_model_fitting_tRansacLeastSquaresPolynomial_h_
#define _rrlib_model_fitting_tRansacLeastSquaresPolynomial_h_

#include "rrlib/model_fitting/tRansacModel.h"
#include "rrlib/model_fitting/tLeastSquaresPolynomial.h"
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
//! Short description of tRansacLeastSquaresPolynomial
/*! A more detailed description of tRansacLeastSquaresPolynomial, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdegree>
class tRansacLeastSquaresPolynomial : public tLeastSquaresPolynomial<Tdegree>,
    public tRansacModel<typename tLeastSquaresPolynomial<Tdegree>::tSample>
{

  typedef model_fitting::tRansacModel<math::tVec2d> tRansacModel;
  typedef model_fitting::tLeastSquaresPolynomial<Tdegree> tLeastSquaresPolynomial;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tLeastSquaresPolynomial::tSample tSample;

  tRansacLeastSquaresPolynomial();

  tRansacLeastSquaresPolynomial(const std::vector<tSample> &measurements,
                                unsigned int max_iterations, float satisfactory_support_ratio, float max_error);

  const size_t MinimalSetSize() const
  {
    return Tdegree + 1;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tRansacLeastSquaresPolynomial";
  }

  virtual const bool FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set);
  virtual const bool FitToSampleIndexSet(const std::vector<size_t> &sample_index_set);
  virtual const float GetSampleError(const tSample &sample) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tRansacLeastSquaresPolynomial.hpp"

#endif
