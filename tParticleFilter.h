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
/*!\file    tParticleFilter.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-19
 *
 * \brief   Contains tParticleFilter
 *
 * \b tParticleFilter
 *
 * A few words for tParticleFilter
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__model_fitting__tParticleFilter_h__
#define __rrlib__model_fitting__tParticleFilter_h__

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
//! Short description of tParticleFilter
/*!
 */
template <typename TConfiguration>
class tParticleFilter
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TConfiguration tConfiguration;

  class tParticle
  {
    friend class tParticleFilter<TConfiguration>;

    tConfiguration configuration;
    double score;

    inline tParticle(const tConfiguration &configuration, double score)
        : configuration(configuration), score(score)
    {}

  public:

    inline const tConfiguration &Configuration() const
    {
      return this->configuration;
    }
    inline const double Score() const
    {
      return this->score;
    }
  };

  explicit tParticleFilter(long int seed = 0);

  virtual ~tParticleFilter() = 0;

  void Initialize(unsigned int number_of_particles,
                  const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tConfiguration &variance);

  void PerformUpdate();

  inline const std::vector<tParticle> &GetParticles() const
  {
    return this->particles;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  struct tSortParticlesByScoreDecreasing
  {
    const bool operator()(const tParticle &a, const tParticle &b) const
    {
      return a.Score() > b.Score();
    }
  };

  unsigned int number_of_particles;
  tConfiguration lower_bound;
  tConfiguration upper_bound;
  tConfiguration variance;

  std::vector<tParticle> particles;

  virtual const char *GetLogDescription() const
  {
    return "tParticleFilter";
  }

  tConfiguration GenerateConfiguration() const;
  tConfiguration GenerateConfiguration(const tConfiguration &center) const;

  inline double CalculateConfigurationScore(const tConfiguration &configuration) const;

  virtual double CalculateConfigurationScoreImplementation(const tConfiguration &configuration) const = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/model_fitting/tParticleFilter.hpp"

#endif