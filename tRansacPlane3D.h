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
/*!\file    tRansacPlane3D.h
 *
 * \author  Tim Braun
 * \author  Tobias Foehst
 *
 * \date    2007-11-16
 *
 * \brief   Contains tRansacPlane3D
 *
 * \b tRansacPlane3D
 *
 * A few words for tRansacPlane3D
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_model_fitting_tRansacPlane3D_h_
#define _rrlib_model_fitting_tRansacPlane3D_h_

#include "rrlib/geometry/tPlane.h"
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
//! Short description of tRansacPlane3D
/*! A more detailed description of tRansacPlane3D, which
    Tobias Foehst hasn't done yet !!
*/
class tRansacPlane3D : public geometry::tPlane3D, public tRansacModel<geometry::tPlane3D::tPoint>
{

  typedef model_fitting::tRansacModel<geometry::tPlane3D::tPoint> tRansacModel;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef geometry::tPlane3D::tPoint tSample;

  tRansacPlane3D(bool local_optimization = false);

  template <typename TIterator>
  tRansacPlane3D(TIterator begin, TIterator end,
                 unsigned int max_iterations, float satisfactory_support_ratio, float max_error,
                 bool local_optimization = false);

  template <typename TSTLContainer>
  explicit tRansacPlane3D(const TSTLContainer &samples,
                          unsigned int max_iterations, float satisfactory_support_ratio, float max_error,
                          bool local_optimization = false);

  const size_t MinimalSetSize() const
  {
    return 3;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tRansacPlane3D";
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

#endif
