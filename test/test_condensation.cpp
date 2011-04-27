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
/*!\file    test_condensation.cpp
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
#include <cstdlib>
#include <iostream>
#include <vector>

#include "rrlib/highgui_wrapper/tWindow.h"

#include "rrlib/math/tVector.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/tCondensation.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::highgui;
using namespace rrlib::math;
using namespace rrlib::model_fitting;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef tVec2d tConfiguration;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

std::vector<tConfiguration> ground_truth;

class tPointFinder : public tCondensation<tConfiguration>
{
private:

  double CalculateConfigurationScore(const tConfiguration &configuration) const
  {
    assert(ground_truth.size() > 0);
    double max_score = 0;
    for (std::vector<tConfiguration>::const_iterator it = ground_truth.begin(); it != ground_truth.end(); ++it)
    {
      max_score = std::max(max_score, 1.0 / (configuration - *it).Length());
    }
    return max_score;
  }
};

int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainConfiguresSubTree(".", true);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainMaxMessageLevel(".", rrlib::logging::eLL_DEBUG_VERBOSE_1);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainPrintsLocation(".", false);

  tWindow &window = tWindow::GetInstance("Condensation Tests", 500, 500);

  ground_truth.resize(1);
  ground_truth[0].Set(0.5, 0.75);

  tPointFinder point_finder;
  point_finder.Initialize(1000, tConfiguration::Zero(), tConfiguration(1, 1), tConfiguration(0.01, 0.01));

  window.Clear();
  window.SetColor(0);
  for (std::vector<tConfiguration>::const_iterator it = ground_truth.begin(); it != ground_truth.end(); ++it)
  {
    window.DrawCircleNormalized(it->X(), it->Y(), 0.01, true);
  }
  window.SetColor(1);
  for (std::vector<tPointFinder::tParticle>::const_iterator it = point_finder.GetParticles().begin(); it != point_finder.GetParticles().end(); ++it)
  {
    window.DrawCircleNormalized(it->configuration.X(), it->configuration.Y(), 0.005, true);
  }
  window.Render();

  for (unsigned int i = 1; i < 400; ++i)
  {
    double x_offset = 0.25 * std::sin(i / (20 * M_PI));
    double y_offset = 0.25 * std::cos(i / (20 * M_PI));
    ground_truth[0].Set(0.5 + x_offset, 0.5 + y_offset);

    point_finder.PerformUpdate();

    window.Clear();
    window.SetColor(0);
    for (std::vector<tConfiguration>::const_iterator it = ground_truth.begin(); it != ground_truth.end(); ++it)
    {
      window.DrawCircleNormalized(it->X(), it->Y(), 0.01, true);
    }
    window.SetColor(1);
    for (std::vector<tPointFinder::tParticle>::const_iterator it = point_finder.GetParticles().begin(); it != point_finder.GetParticles().end(); ++it)
    {
      window.DrawCircleNormalized(it->configuration.X(), it->configuration.Y(), 0.005, true);
    }
    window.Render();
  }

  ground_truth.resize(2);

  for (unsigned int i = 0; i < 800; ++i)
  {
    double x_offset = 0.25 * std::sin(i / (20 * M_PI));
    double y_offset = 0.25 * std::cos(i / (20 * M_PI));
    ground_truth[0].Set(0.5 + x_offset, 0.5 + y_offset);
    ground_truth[1].Set(0.3 - x_offset, 0.3 + y_offset);

    point_finder.PerformUpdate();

    window.Clear();
    window.SetColor(0);
    for (std::vector<tConfiguration>::const_iterator it = ground_truth.begin(); it != ground_truth.end(); ++it)
    {
      window.DrawCircleNormalized(it->X(), it->Y(), 0.01, true);
    }
    window.SetColor(1);
    for (std::vector<tPointFinder::tParticle>::const_iterator it = point_finder.GetParticles().begin(); it != point_finder.GetParticles().end(); ++it)
    {
      window.DrawCircleNormalized(it->configuration.X(), it->configuration.Y(), 0.005, true);
    }
    window.Render();
  }

  window.Render();
  std::cout << "OK" << std::endl;

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
