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

tConfiguration ground_truth;

class tPointFinder : public tCondensation<tConfiguration>
{
private:

  float CalculateConfigurationScore(const tConfiguration &configuration) const
  {
    return 1.0 / (configuration - ground_truth).Length();
  }
};

int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainConfiguresSubTree(".", true);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainMaxMessageLevel(".", rrlib::logging::eLL_DEBUG_VERBOSE_3);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainPrintsLocation(".", false);

  tWindow &window = tWindow::GetInstance("Condensation Tests", 500, 500);

  ground_truth.Set(0.5, 0.5);

  window.SetColor(0);
  window.DrawCircleNormalized(ground_truth.X(), ground_truth.Y(), 0.01, true);
  window.Render();

  tPointFinder point_finder;

  point_finder.Initialize(50, tConfiguration::Zero(), tConfiguration(1, 1), tConfiguration(0.01, 0.01));

  window.Clear();
  window.SetColor(0);
  window.DrawCircleNormalized(ground_truth.X(), ground_truth.Y(), 0.01, true);
  window.SetColor(1);
  for (std::vector<tPointFinder::tParticle>::const_iterator it = point_finder.GetParticles().begin(); it != point_finder.GetParticles().end(); ++it)
  {
    window.DrawCircleNormalized(it->configuration.X(), it->configuration.Y(), 0.005, true);
  }
  window.Render();

  for (unsigned int i = 0; i < 10; ++i)
  {
    point_finder.PerformUpdate();

    window.Clear();
    window.SetColor(0);
    window.DrawCircleNormalized(ground_truth.X(), ground_truth.Y(), 0.01, true);
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
