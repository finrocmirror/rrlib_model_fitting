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
/*!\file    tParticleFilter.hpp
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
#include <algorithm>

#include "rrlib/logging/messages.h"

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
// tParticleFilter constructors
//----------------------------------------------------------------------
template <typename TConfiguration>
tParticleFilter<TConfiguration>::tParticleFilter(long int seed)
  : number_of_particles(0),
    rng_engine(seed)
{}

//----------------------------------------------------------------------
// tParticleFilter destructor
//----------------------------------------------------------------------
template <typename TConfiguration>
tParticleFilter<TConfiguration>::~tParticleFilter()
{}

//----------------------------------------------------------------------
// tParticleFilter Initialize
//----------------------------------------------------------------------
template <typename TConfiguration>
void tParticleFilter<TConfiguration>::Initialize(unsigned int number_of_particles,
    const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tCovariance &covariance, double resampling_ratio)
{
  assert(number_of_particles > 0);
  this->number_of_particles = number_of_particles;
  this->lower_bound = lower_bound;
  this->upper_bound = upper_bound;
  this->SetCovariance(covariance);
  this->SetResamplingRatio(resampling_ratio);

  this->particles.reserve(this->number_of_particles);
}

template <typename TConfiguration>
void tParticleFilter<TConfiguration>::Initialize(unsigned int number_of_particles,
    const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tConfiguration &variance, double resampling_ratio)
{
  this->Initialize(number_of_particles, lower_bound, upper_bound, tCovariance::Diagonal(variance), resampling_ratio);
}

//----------------------------------------------------------------------
// tParticleFilter GenerateConfiguration
//----------------------------------------------------------------------
template <typename TConfiguration>
TConfiguration tParticleFilter<TConfiguration>::GenerateConfiguration(const tConfiguration &center) const
{
  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "Generating particle around ", center, " with covariance ", this->multivariate_normal_distribution.Covariance(), ".");
  while (true)
  {
    tConfiguration configuration = center + this->multivariate_normal_distribution(this->rng_engine);

    bool accept = true;
    for (size_t i = 0; i < TConfiguration::cDIMENSION; ++i)
    {
      if (this->lower_bound[i] > configuration[i] || configuration[i] > this->upper_bound[i])
      {
        RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "Rejecting ", configuration);
        accept = false;
        break;
      }
    }
    if (accept)
    {
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "Accepting ", configuration);
      return configuration;
    }
  }
}

//----------------------------------------------------------------------
// tParticleFilter CalculateConfigurationScore
//----------------------------------------------------------------------
template <typename TConfiguration>
double tParticleFilter<TConfiguration>::CalculateConfigurationScore(const tConfiguration &configuration) const
{
  double score = this->CalculateConfigurationScoreImplementation(configuration);
  assert(score >= 0.0);
  return score;
}

//----------------------------------------------------------------------
// tParticleFilter PerformUpdate
//----------------------------------------------------------------------
template <typename TConfiguration>
void tParticleFilter<TConfiguration>::PerformUpdate()
{
  if (this->particles.size() < this->number_of_particles)
  {
    for (size_t i = this->particles.size(); i < this->number_of_particles; ++i)
    {
      tConfiguration configuration;
      for (size_t k = 0; k < tConfiguration::cDIMENSION; ++k)
      {
        configuration[k] = std::uniform_real_distribution<typename tConfiguration::tElement>(this->lower_bound[k], this->upper_bound[k])(this->rng_engine);
      }
      double score = this->CalculateConfigurationScore(configuration);
      this->particles.push_back(tParticle(configuration, score));
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "Generated new particle with configuration ", this->particles.back().Configuration(), " and score ", this->particles.back().Score());
    }
    std::sort(this->particles.begin(), this->particles.end(), [](const tParticle & a, const tParticle & b)
    {
      return a.Score() > b.Score();
    });
  }

  double total_score = 0;
  for (typename std::vector<tParticle>::iterator it = this->particles.begin(); it != this->particles.end(); ++it)
  {
    total_score += it->Score();
  }

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Total score: ", total_score);

  if (total_score > 0.0)
  {
    for (typename std::vector<tParticle>::iterator it = this->particles.begin(); it != this->particles.end(); ++it)
    {
      it->score /= total_score;
    }
  }

  size_t resampling_size = this->resampling_ratio * this->number_of_particles;

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Resampling ", resampling_size, " particles...");

  std::vector<tConfiguration> new_configurations;
  new_configurations.reserve(this->number_of_particles);
  for (size_t i = 0; i < resampling_size; ++i)
  {
    size_t number_of_clones = this->number_of_particles * this->particles[i].score;

    RRLIB_LOG_PRINT(DEBUG_VERBOSE_2, "Resampling ", number_of_clones, " particles from ", this->particles[i].configuration, " with score ", this->particles[i].score, ".");

    if (number_of_clones == 0 || new_configurations.size() + number_of_clones > resampling_size)
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
    this->particles[i].score = this->CalculateConfigurationScore(this->particles[i].configuration);
  }

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_2, "Resampled ", new_configurations.size(), " particles.");

  typename std::vector<tParticle>::iterator new_end = this->particles.begin();
  std::advance(new_end, new_configurations.size());
  this->particles.erase(new_end, this->particles.end());

  std::sort(this->particles.begin(), this->particles.end(), [](const tParticle & a, const tParticle & b)
  {
    return a.Score() > b.Score();
  });
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
