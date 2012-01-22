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
/*!\file    tRansacPlane3D.hpp
 *
 * \author  Tim Braun
 * \author  Tobias Foehst
 *
 * \date    2007-11-16
 *
 */
//----------------------------------------------------------------------
#include "rrlib/model_fitting/tRansacPlane3D.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>
#include <cv.h>

#include "rrlib/util/join.h"

#include "rrlib/logging/messages.h"

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

//----------------------------------------------------------------------
// tRansacPlane3D constructors
//----------------------------------------------------------------------
tRansacPlane3D::tRansacPlane3D(bool local_optimization)
  : tRansacModel(local_optimization)
{}

//----------------------------------------------------------------------
// tRansacPlane3D SetNormalConstraint
//----------------------------------------------------------------------
void tRansacPlane3D::SetNormalConstraint(const math::tVec3d &direction, math::tAngleRadUnsigned max_angle_distance)
{
  this->normal_constraint.active = true;
  this->normal_constraint.direction = direction.Normalized();
  this->normal_constraint.max_angle_distance = max_angle_distance;
}

//----------------------------------------------------------------------
// tRansacPlane3D SetPointConstraint
//----------------------------------------------------------------------
void tRansacPlane3D::SetPointConstraint(const geometry::tPlane3D::tPoint &reference_point, double min_distance, double max_distance)
{
  this->point_constraint.active = true;
  this->point_constraint.reference_point = reference_point;
  this->point_constraint.min_distance = min_distance;
  this->point_constraint.max_distance = max_distance;
}

//----------------------------------------------------------------------
// tRansacPlane3D ClearNormalConstraint
//----------------------------------------------------------------------
void tRansacPlane3D::ClearNormalConstraint()
{
  this->normal_constraint.active = false;
}

//----------------------------------------------------------------------
// tRansacPlane3D ClearPointConstraint
//----------------------------------------------------------------------
void tRansacPlane3D::ClearPointConstraint()
{
  this->point_constraint.active = false;
}

//----------------------------------------------------------------------
// tRansacPlane3D FitToMinimalSampleIndexSet
//----------------------------------------------------------------------
const bool tRansacPlane3D::FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  const tSample &p1(this->Samples()[sample_index_set[0]]);
  const tSample &p2(this->Samples()[sample_index_set[1]]);
  const tSample &p3(this->Samples()[sample_index_set[2]]);

  tSample p1_p2(p2 - p1);
  tSample p1_p3(p3 - p1);
  tSample p2_p3(p3 - p2);

  // ensure distinct points
  if (p1_p2.IsZero() || p1_p3.IsZero() || p2_p3.IsZero())
  {
    return false;
  }

  // ensure that chosen points are not colinear // FIXME: re-think this condition
//  if (p1_p2 * p1_p3 > 0.999)
//  {
//    return false;
//  }

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Using points: ", p1, ", ", p2, ", ", p3);
  this->Set(p1, p2, p3);
  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Plane: (", this->Support(), ", ", this->Normal(), ")");

  if (this->normal_constraint.active)
  {
    if (EnclosedAngle(this->Normal(), this->normal_constraint.direction) > math::tAngleRadUnsigned(math::tAngleDegUnsigned(90)))
    {
      this->Set(this->Support(), -this->Normal());
    }
  }

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_1, "Checking constraints");
  if (!this->CheckConstraints())
  {
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_1, "Constraints violated!");
    return false;
  }

  return true;
}

//----------------------------------------------------------------------
// tRansacPlane3D FitToSampleIndexSet
//----------------------------------------------------------------------
const bool tRansacPlane3D::FitToSampleIndexSet(const std::vector<size_t> &sample_index_set)
{
  // perform PCA
  tPoint center_of_gravity;
  for (std::vector<size_t>::const_iterator it = sample_index_set.begin(); it != sample_index_set.end(); ++it)
  {
    center_of_gravity += this->Samples()[*it];
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Using sample ", this->Samples()[*it]);
  }
  center_of_gravity /= sample_index_set.size();

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Center of gravity: ", center_of_gravity);

  double covariance[9];
  std::memset(covariance, 0, sizeof(covariance));
  CvMat cv_covariance = cvMat(3, 3, CV_64FC1, covariance);
  for (std::vector<size_t>::const_iterator it = sample_index_set.begin(); it != sample_index_set.end(); ++it)
  {
    tSample centered_point = this->Samples()[*it] - center_of_gravity;

    covariance[0] += centered_point.X() * centered_point.X();
    covariance[1] = covariance[3] += centered_point.X() * centered_point.Y();
    covariance[2] = covariance[6] += centered_point.X() * centered_point.Z();
    covariance[4] += centered_point.Y() * centered_point.Y();
    covariance[5] = covariance[7] += centered_point.Y() * centered_point.Z();
    covariance[8] += centered_point.Z() * centered_point.Z();
  }

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "Covariance matrix: [ ", util::Join(covariance, covariance + 9, ", "), "]");

  double s[9];
  double u[9];
  CvMat cv_s = cvMat(3, 3, CV_64FC1, s);
  CvMat cv_u = cvMat(3, 3, CV_64FC1, u);
  cvSVD(&cv_covariance, &cv_s, &cv_u, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

  // use weakest component as plane normal
  math::tVec3d normal(u[6], u[7], u[8]);

  // the current normal was checked against the constraints. maybe the normal from the SVD changed its direction
  this->Set(center_of_gravity, normal * this->Normal() < 0 ? -normal : normal);
  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_3, "After fitting: (", this->Support(), ", ", this->Normal(), ")");

  RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_1, "Checking constraints");
  if (!this->CheckConstraints())
  {
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_1, "Constraints violated!");
    return false;
  }

  return true;
}

//----------------------------------------------------------------------
// tRansacPlane3D GetSampleError
//----------------------------------------------------------------------
const double tRansacPlane3D::GetSampleError(const tSample &sample) const
{
  return this->GetDistanceToPoint(sample);
}

//----------------------------------------------------------------------
// tRansacPlane3D CheckConstraints
//----------------------------------------------------------------------
const bool tRansacPlane3D::CheckConstraints() const
{
  if (this->normal_constraint.active)
  {
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Checking normal constraint:");

    if (EnclosedAngle(this->Normal(), this->normal_constraint.direction) > this->normal_constraint.max_angle_distance)
    {
      RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Failed!");
      return false;
    }
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "OK.");
  }

  if (this->point_constraint.active)
  {
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Checking point constraint:");

    double distance = this->GetDistanceToPoint(this->point_constraint.reference_point);
    if (distance < this->point_constraint.min_distance || distance > this->point_constraint.max_distance)
    {
      RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "Failed!");
      return false;
    }
    RRLIB_LOG_PRINT(logging::eLL_DEBUG_VERBOSE_2, "OK.");
  }
  return true;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
