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
#ifndef __rrlib__model_fitting__tRansacLeastSquaresPolynomial_h__
#define __rrlib__model_fitting__tRansacLeastSquaresPolynomial_h__

#include "rrlib/model_fitting/tLeastSquaresPolynomial.h"
#include "rrlib/model_fitting/tRansacModel.h"
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

  typedef model_fitting::tRansacModel<typename tLeastSquaresPolynomial<Tdegree>::tSample> tRansacModel;
  typedef model_fitting::tLeastSquaresPolynomial<Tdegree> tLeastSquaresPolynomial;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tLeastSquaresPolynomial::tSample tSample;

  explicit tRansacLeastSquaresPolynomial(bool local_optimization = false);

  template <typename TIterator>
  tRansacLeastSquaresPolynomial(TIterator begin, TIterator end,
                                unsigned int max_iterations = 50, double satisfactory_support_ratio = 1.0, double max_error = 1E-6,
                                bool local_optimization = false);

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
  virtual const double GetSampleError(const tSample &sample) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tRansacLeastSquaresPolynomial.hpp"

#endif
