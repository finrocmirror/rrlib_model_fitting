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
tCondensation<TConfiguration>::tCondensation()
    : number_of_particles(0),
    condensation(0),
    random_number_generator(cvRNG())
{}

//----------------------------------------------------------------------
// tCondensation destructor
//----------------------------------------------------------------------
template <typename TConfiguration>
tCondensation<TConfiguration>::~tCondensation()
{
  if (this->condensation)
  {
    cvReleaseConDensation(&this->condensation);
  }
}

//----------------------------------------------------------------------
// tCondensation Initialize
//----------------------------------------------------------------------
template <typename TConfiguration>
void tCondensation<TConfiguration>::Initialize(unsigned int number_of_particles,
    const tConfiguration &lower_bound, const tConfiguration &upper_bound, const tConfiguration &variance)
{
  assert(number_of_particles > 0);
  this->number_of_particles = number_of_particles;

  if (this->condensation)
  {
    cvReleaseConDensation(&this->condensation);
    this->condensation = NULL;
  }
  assert(!this->condensation);
  this->condensation = cvCreateConDensation(tConfiguration::cDIMENSION, tConfiguration::cDIMENSION, this->number_of_particles);
  assert(this->condensation);

  this->particles.reserve(this->number_of_particles);

  float cv_lower_bound_buffer[tConfiguration::cDIMENSION];
  float cv_upper_bound_buffer[tConfiguration::cDIMENSION];
  CvMat cv_lower_bound = cvMat(tConfiguration::cDIMENSION, 1, CV_MAT32F, cv_lower_bound_buffer);
  CvMat cv_upper_bound = cvMat(tConfiguration::cDIMENSION, 1, CV_MAT32F, cv_upper_bound_buffer);
  for (size_t i = 0; i < tConfiguration::cDIMENSION; ++i)
  {
    cvmSet(&cv_lower_bound, i, 0, lower_bound[i]);
    cvmSet(&cv_upper_bound, i, 0, upper_bound[i]);
  }
  cvConDensInitSampleSet(this->condensation, &cv_lower_bound, &cv_upper_bound);

  assert(static_cast<size_t>(this->condensation->DP) == tConfiguration::cDIMENSION);
  for (size_t i = 0; i < tConfiguration::cDIMENSION; ++i)
  {
    cvRandInit(&this->condensation->RandS[i], -variance[i], variance[i], i);
  }

  CvMat dynamic_matrix = cvMat(tConfiguration::cDIMENSION, tConfiguration::cDIMENSION, CV_MAT32F, this->condensation->DynamMatr);
  cvmSetIdentity(&dynamic_matrix);

  this->UpdateParticles();

  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Initialized with ", this->number_of_particles, " particles.");
}

//----------------------------------------------------------------------
// tCondensation UpdateParticles
//----------------------------------------------------------------------
template <typename TConfiguration>
void tCondensation<TConfiguration>::UpdateParticles()
{
  this->particles.clear();
  for (unsigned int i = 0; i < this->number_of_particles; ++i)
  {
    tParticle particle;
    particle.configuration = tConfiguration(this->condensation->flSamples[i]);
    particle.score = CalculateConfigurationScore(particle.configuration);

    this->condensation->flConfidence[i] = std::max<float>(0.0, particle.score);

    this->particles.push_back(particle);
  }
}

//----------------------------------------------------------------------
// tCondensation PerformUpdate
//----------------------------------------------------------------------
template <typename TConfiguration>
const bool tCondensation<TConfiguration>::PerformUpdate()
{
  assert(this->condensation && static_cast<unsigned int>(this->condensation->SamplesNum) == this->number_of_particles);

  cvConDensUpdateByTime(this->condensation);

  this->UpdateParticles();

//  for (unsigned int i = 0; i < this->number_of_particles; ++i)
//  {
//    tParticle particle;
//    for (size_t i = 0; i < tConfiguration::cDIMENSION; ++i)
//    {
//      particle.configuration = tConfiguration(this->condensation->flSamples[i]);
//    }

//    // reinitialize if out of bounds
//    if (my_model.IsAnwhereLowerThan(this->minimum_value_model) || my_model.IsAnwhereHigherThan(this->maximum_value_model))
//    {
//      ++reinited;
//      ReinitSample(this->condensation->flSamples[i]);
//      my_model.SetParametersFromArray(this->condensation->flSamples[i]);
//    }

//    // update top n models
//    if (top_n_models.size() < top_n)
//    {
//      // normalize model center
//      my_model.NormalizeToCenterPosition(center_of_point_cloud_x, center_of_point_cloud_y);
//
//      top_n_models.insert(std::make_pair(model_score, tVector<5, float> (&my_model[0])));
//    }
//    else
//    {
//      // check if current model is better than worst of top n
//      if (top_n_least_score < model_score)
//      {
//        // normalize model center
//        my_model.NormalizeToCenterPosition(center_of_point_cloud_x, center_of_point_cloud_y);
//
//        // replace first model with lowest score
//        top_n_models.erase(top_n_models.begin());
//        top_n_models.insert(std::make_pair(model_score, tVector<5, float> (&my_model[0])));
//        top_n_least_score = top_n_models.begin()->first;
//      }
//    }

//  }

  return true;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
