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
/*!\file    rrlib/model_fitting/examples/icp.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2017-04-12
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/highgui_wrapper/tWindow.h"

#include "rrlib/logging/configuration.h"

#define RRLIB_MODEL_FITTING_DEBUG_ICP
#include "rrlib/model_fitting/tIterativeClosestPoint.h"
#include "rrlib/math/tPose2D.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::SetDomainMaxMessageLevel(".", rrlib::logging::tLogLevel::DEBUG_VERBOSE_3);
//  rrlib::logging::SetDomainPrintsLocation(".", false);

  std::vector<rrlib::math::tVec2d> model;
  model.emplace_back(270, 380);
  model.emplace_back(30, 420);
  model.emplace_back(10, 270);
  model.emplace_back(150, 30);
  model.emplace_back(850, 170);
  model.emplace_back(20, 350);
  model.emplace_back(40, 420);
  model.emplace_back(60, 260);
  model.emplace_back(650, 200);
  model.emplace_back(90, 250);
  model.emplace_back(70, 110);
  model.emplace_back(450, 380);
  model.emplace_back(50, 450);
  model.emplace_back(750, 120);
  model.emplace_back(550, 320);
  model.emplace_back(80, 140);
  model.emplace_back(350, 430);

  rrlib::math::tPose2D real_transform(200, 500, rrlib::math::tAngleDeg(50));

  std::vector<rrlib::math::tVec2d> data;
  for (auto & m : model)
  {
    data.emplace_back(rrlib::math::tPose2D(m).GetPoseInParentFrame(real_transform).Position());
  }

  rrlib::model_fitting::tIterativeClosestPoint2D icp(model.begin(), model.end(), data.begin(), data.end());

  RRLIB_LOG_PRINT(DEBUG, "transformation: ", icp.Transformation());

  rrlib::highgui::tWindow::ReleaseAllInstances();

  model.clear();
  model.emplace_back(270, 380);
  model.emplace_back(30, 420);
  model.emplace_back(10, 270);
  model.emplace_back(150, 30);
  model.emplace_back(850, 170);
  model.emplace_back(20, 350);
  model.emplace_back(40, 420);
  model.emplace_back(60, 260);
  model.emplace_back(650, 200);
//  model.emplace_back(90, 250);
  model.emplace_back(70, 110);
  model.emplace_back(450, 380);
  model.emplace_back(50, 450);
  model.emplace_back(750, 120);
  model.emplace_back(550, 320);
  model.emplace_back(80, 140);
  model.emplace_back(350, 430);

  data.clear();
  data.emplace_back(270, 380);
  data.emplace_back(30, 420);
//  data.emplace_back(10, 270);
  data.emplace_back(150, 30);
  data.emplace_back(850, 170);
  data.emplace_back(20, 350);
  data.emplace_back(40, 420);
  data.emplace_back(60, 260);
//  data.emplace_back(650, 200);
  data.emplace_back(90, 250);
  data.emplace_back(70, 110);
  data.emplace_back(450, 380);
  data.emplace_back(50, 450);
  data.emplace_back(750, 120);
//  data.emplace_back(550, 320);
  data.emplace_back(80, 140);
  data.emplace_back(350, 430);
  for (auto & d : data)
  {
    d = rrlib::math::tPose2D(d).GetPoseInParentFrame(real_transform).Position();
  }

  icp.SetModel(model.begin(), model.end());
  icp.SetData(data.begin(), data.end());
  icp.DoICP();

  RRLIB_LOG_PRINT(DEBUG, "transformation: ", icp.Transformation());

  rrlib::highgui::tWindow::ReleaseAllInstances();
}
