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
/*!\file    rrlib/model_fitting/tests/icp.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2017-04-25
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

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
class TestICP : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestICP);
  RRLIB_UNIT_TESTS_ADD_TEST(RelativeMotion);
  RRLIB_UNIT_TESTS_ADD_TEST(EmptyPointSets);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void RelativeMotion()
  {
    std::vector<math::tVec2d> world_data;
    world_data.emplace_back(200, 150);
    world_data.emplace_back(300, 300);
    world_data.emplace_back(500, 100);

    math::tPose2D sensor_pose(10, 10, math::tAngleDeg(20));

    std::vector<math::tVec2d> data;
    for (auto & d : world_data)
    {
      data.emplace_back(math::tPose2D(d).GetPoseInLocalFrame(sensor_pose).Position());
    }

    math::tPose2D motion(5, 0, math::tAngleDeg(10));
    auto new_sensor_pose = motion.GetPoseInParentFrame(sensor_pose);

    std::vector<math::tVec2d> model;
    for (auto & d : world_data)
    {
      model.emplace_back(math::tPose2D(d).GetPoseInLocalFrame(new_sensor_pose).Position());
    }

    tIterativeClosestPoint2D icp;
    icp.SetModel(model.begin(), model.end());
    icp.SetData(data.begin(), data.end());
    RRLIB_UNIT_TESTS_ASSERT(icp.DoICP(1E-13));

    math::tPose2D result(icp.Transformation());
    std::ostringstream output;
    output << std::setprecision(10) << result << " != " << motion;

    RRLIB_UNIT_TESTS_ASSERT_MESSAGE(output.str(), IsEqual(result, motion, 1E-6));
  }

  void EmptyPointSets()
  {
    std::vector<math::tVec2d> data {math::tVec2d(0, 0)};
    std::vector<math::tVec2d> model {math::tVec2d(1, 1)};;

    tIterativeClosestPoint2D icp;
    icp.SetModel(model.begin(), model.begin());
    icp.SetData(data.begin(), data.begin());
    RRLIB_UNIT_TESTS_ASSERT(!icp.DoICP());

    icp.SetModel(model.begin(), model.end());
    RRLIB_UNIT_TESTS_ASSERT(!icp.DoICP());

    icp.SetModel(model.begin(), model.begin());
    icp.SetData(data.begin(), data.end());
    RRLIB_UNIT_TESTS_ASSERT(!icp.DoICP());

    RRLIB_UNIT_TESTS_EXCEPTION(tIterativeClosestPoint2D(model.begin(), model.begin(), data.begin(), data.begin()), std::runtime_error);
    RRLIB_UNIT_TESTS_EXCEPTION(tIterativeClosestPoint2D(model.begin(), model.end(), data.begin(), data.begin()), std::runtime_error);
    RRLIB_UNIT_TESTS_EXCEPTION(tIterativeClosestPoint2D(model.begin(), model.begin(), data.begin(), data.end()), std::runtime_error);
  }

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestICP);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
