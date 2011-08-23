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
/*!\file    tCondensation.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-19
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
// tCondensation constructors
//----------------------------------------------------------------------
template <typename TConfiguration>
tCondensation<TConfiguration>::tCondensation(long int seed)
    : number_of_particles(0)
{
  srand48(seed);
}

//----------------------------------------------------------------------
// tCondensation destructor
//----------------------------------------------------------------------
template <typename TConfiguration>
tCondensation<TConfiguration>::~tCondensation()
{}

//----------------------------------------------------------------------
// tCondensation Initialize
//----------------------------------------------------------------------
template <typename TConfiguration>
void tCondensation<TConfiguration>::Initialize(unsigned int number_of_particles,
    const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tConfiguration &variance)
{
  assert(number_of_particles > 0);
  this->number_of_particles = number_of_particles;
  this->lower_bound = lower_bound;
  this->upper_bound = upper_bound;
  this->variance = variance;

  this->particles.resize(this->number_of_particles);
  for (size_t i = 0; i < this->number_of_particles; ++i)
  {
    this->particles[i].configuration = SchurProduct(this->GenerateConfiguration(), this->upper_bound - this->lower_bound) + this->lower_bound;
    this->particles[i].score = std::max(0.0, this->CalculateConfigurationScore(this->particles[i].configuration));
    RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_3, "Generated new particle with configuration ", this->particles[i].configuration, ".");
  }

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Initialized with ", this->number_of_particles, " particles.");
}

//----------------------------------------------------------------------
// tCondensation GenerateConfiguration
//----------------------------------------------------------------------
template <typename TConfiguration>
TConfiguration tCondensation<TConfiguration>::GenerateConfiguration() const
{
  tConfiguration configuration;
  for (size_t i = 0; i < tConfiguration::cDIMENSION; ++i)
  {
    configuration[i] = drand48();
  }
  return configuration;
}

template <typename TConfiguration>
TConfiguration tCondensation<TConfiguration>::GenerateConfiguration(const tConfiguration &center) const
{
  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Generating particle around ", center, " with variance ", this->variance, ".");
  return center - this->variance + SchurProduct(this->GenerateConfiguration(), 2 * this->variance);
}

//----------------------------------------------------------------------
// tCondensation PerformUpdate
//----------------------------------------------------------------------
template <typename TConfiguration>
void tCondensation<TConfiguration>::PerformUpdate()
{
  std::sort(this->particles.begin(), this->particles.end(), tSortParticlesByScoreDecreasing());

  double total_score = 0;
  for (typename std::vector<tParticle>::iterator it = this->particles.begin(); it != this->particles.end(); ++it)
  {
    total_score += it->score;
  }
  for (typename std::vector<tParticle>::iterator it = this->particles.begin(); it != this->particles.end(); ++it)
  {
    it->score /= total_score;
  }

  size_t resampling_size = 0.90 * this->number_of_particles;

  std::vector<tConfiguration> new_configurations;
  new_configurations.reserve(this->number_of_particles);
  for (size_t i = 0; i < resampling_size; ++i)
  {
    size_t number_of_clones = this->number_of_particles * this->particles[i].score;

    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Resampling ", number_of_clones, " particles from ", this->particles[i].configuration, " with score ", this->particles[i].score, ".");

    if (resampling_size == 0 || new_configurations.size() + number_of_clones > resampling_size)
    {
      break;
    }
    for (size_t k = 0; k < number_of_clones; ++k)
    {
      new_configurations.push_back(this->GenerateConfiguration(this->particles[i].configuration));
    }
  }

  for (size_t i = 0; i < new_configurations.size(); ++i)
  {
    this->particles[i].configuration = new_configurations[i];
    this->particles[i].score = std::max(0.0, this->CalculateConfigurationScore(this->particles[i].configuration));
  }

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Resampled ", new_configurations.size(), " particles.");

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Generating ", this->number_of_particles - new_configurations.size(), " new particles.");

  for (size_t i = new_configurations.size(); i < this->number_of_particles; ++i)
  {
    this->particles[i].configuration = SchurProduct(this->GenerateConfiguration(), this->upper_bound - this->lower_bound) + this->lower_bound;
    this->particles[i].score = std::max(0.0, this->CalculateConfigurationScore(this->particles[i].configuration));
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
