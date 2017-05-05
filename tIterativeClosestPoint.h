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
/*!\file    rrlib/model_fitting/tIterativeClosestPoint.h
 *
 * \author  Tobias Föhst
 *
 * \date    2017-04-12
 *
 * \brief   Contains tIterativeClosestPoint
 *
 * \b tIterativeClosestPoint
 *
 * Implements the ICP algorithm for point cloud registration
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__tIterativeClosestPoint_h__
#define __rrlib__model_fitting__tIterativeClosestPoint_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
//! SHORT_DESCRIPTION
/*!
 * Implements the ICP algorithm for point cloud registration
 */
template <size_t Tdimension>
class tIterativeClosestPoint
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  using tSample = math::tVector<Tdimension>;
  using tRotation = math::tMatrix<Tdimension, Tdimension>;
  using tTransformation = math::tMatrix < Tdimension + 1, Tdimension + 1 >;

  /*! Initializes a fresh ICP instance
   *
   */
  tIterativeClosestPoint();

  /*! Executes ICP on given model and data samples
   *
   * \param model_begin                        Iterator marking begin of model samples
   * \param model_end                          Iterator marking end of model samples
   * \param data_begin                         Iterator marking begin of data samples
   * \param data_end                           Iterator marking end of data samples
   * \param sufficient_improvement_threshold   If the errors of two successive iterations are lower the algorithm terminates
   * \param max_iterations                     Maximum number of iterations until the algorithm terminates without further convergence
   */
  template <typename TModelIterator, typename TDataIterator>
  tIterativeClosestPoint(TModelIterator model_begin, TModelIterator model_end, TDataIterator data_begin, TDataIterator data_end,
                         double sufficient_improvement_threshold = 1E-10, unsigned int max_iterations = 1000);

  /*! Reset ICP instance to be re-run (with different samples)
   */
  void Reset();

  /*! Set model for next execution
   *
   * \param begin   Iterator marking begin of model samples
   * \param end     Iterator marking end of model samples
   */
  template <typename TIterator>
  inline void SetModel(TIterator begin, TIterator end);

  /*! Set data for next execution
   *
   * \param begin   Iterator marking begin of data samples
   * \param end     Iterator marking end of data samples
   */
  template <typename TIterator>
  inline void SetData(TIterator begin, TIterator end);

  /*! Execute a run of ICP on the previously set model and data samples
   *
   * \param sufficient_improvement_threshold   If the errors of two successive iterations are lower the algorithm terminates
   * \param max_iterations                     Maximum number of iterations until the algorithm terminates without further convergence
   *
   * \returns false when input data was not suitable for running ICP, true otherwise indicating success
   */
  bool DoICP(double sufficient_improvement_threshold = 1E-10, unsigned int max_iterations = 500);

  /*! Access the originally stored model samples
   *
   * \returns Vector of original model samples
   */
  inline const std::vector<tSample> &Model() const;

  /*! Access the stored data samples
   *
   * \returns Vector of data samples
   */
  inline const std::vector<tSample> &Data() const;

  /*! Access the resulting correspondence pairs
   *
   * \returns Vector of paired indices to model and data samples
   */
  inline const std::vector<std::pair<size_t, size_t>> &CorrespondencePairs() const;

  /*! The mean squared error of the resulting registration
   *
   * \returns The remaining error after optimization
   */
  inline double Error() const;

  /*! The resulting transformation for registration of the stored model samples to the stored data samples
   *
   * \returns A homogeneous transformation matrix containing rotation and translation
   */
  inline const tTransformation &Transformation() const;

  /*! The transformed model samples
   *
   * \returns A vector of model samples after applying the calculated transformation
   */
  inline const std::vector<tSample> &TransformedModel() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::vector<tSample> model;
  std::vector<tSample> data;

  std::vector<std::pair<size_t, size_t>> correspondence_pairs;
  double mse;
  tTransformation transformation;

  mutable std::vector<tSample> transformed_model;

  void FindCorrespondencePairs(std::vector<double> &distances, const std::vector<tSample> &model);

  void FilterCorrespondencePairs(std::vector<double> &distances);

  void FilterCorrespondencePairs(std::vector<double> &distances, double percentage);

  tRotation FindRotation(const std::vector<tSample> &model);

};

using tIterativeClosestPoint2D = tIterativeClosestPoint<2>;
using tIterativeClosestPoint3D = tIterativeClosestPoint<3>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tIterativeClosestPoint.hpp"

#endif
