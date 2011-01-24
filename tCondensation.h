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
/*!\file    tCondensation.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-19
 *
 * \brief   Contains tCondensation
 *
 * \b tCondensation
 *
 * A few words for tCondensation
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_model_fitting_tCondensation_h_
#define _rrlib_model_fitting_tCondensation_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

#include <cv.h>

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
//! Short description of tCondensation
/*!
 */
template <typename TConfiguration>
class tCondensation
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TConfiguration tConfiguration;
  struct tParticle
  {
    tConfiguration configuration;
    float score;
  };

  tCondensation();

  virtual ~tCondensation() = 0;

  void Initialize(unsigned int number_of_particles,
                  const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tConfiguration &variance);

  const bool PerformUpdate();

  inline const std::vector<tParticle> &GetParticles() const
  {
    return this->particles;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  unsigned int number_of_particles;
  std::vector<tParticle> particles;

  CvConDensation *condensation;
  CvRNG random_number_generator;

  virtual const char *GetLogDescription() const
  {
    return "tCondensation";
  }

  void UpdateParticles();

  virtual float CalculateConfigurationScore(const tConfiguration &configuration) const = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tCondensation.hpp"

#endif
