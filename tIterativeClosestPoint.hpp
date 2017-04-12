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
/*!\file    rrlib/model_fitting/tIterativeClosestPoint.hpp
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
#include <cv.h>

#ifdef _LIB_RRLIB_HIGHGUI_WRAPPER_PRESENT_
# ifdef RRLIB_MODEL_FITTING_DEBUG_ICP
#  include "rrlib/highgui_wrapper/tWindow.h"
#  include "rrlib/geometry/tBoundingBox.h"
# endif
#else
# undef RRLIB_MODEL_FITTING_DEBUG_ICP
#endif

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

namespace
{

//----------------------------------------------------------------------
// Visualize
//----------------------------------------------------------------------
template <typename TModel, typename TData, typename TCorrespondencePairs>
void Visualize(const TModel &model, const TData &data, const TCorrespondencePairs &correspondence_pairs)
{
#ifdef RRLIB_MODEL_FITTING_DEBUG_ICP
  auto &window = highgui::tWindow::GetInstance("Debug ICP");

  window.Clear();
  window.SetColor(1);
  for (const auto & p : model)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 3, true);
  }
  window.SetColor(0);
  for (auto & p : data)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 2, true);
  }
  window.SetColor(4);
  for (const auto & c : correspondence_pairs)
  {
    auto &m = model[c.first];
    auto &d = data[c.second];
    window.DrawLineShifted(m.X(), m.Y(), d.X(), d.Y());
  }
  window.Render(1);
#endif
}

//----------------------------------------------------------------------
// CalculateMSE
//----------------------------------------------------------------------
double CalculateMSE(const std::vector<double> &distances)
{
  double result = 0;
  for (const auto d : distances)
  {
    result += d * d;
  }
  return result /= distances.size();
}


//----------------------------------------------------------------------
// CreateTransformationMatrix
//----------------------------------------------------------------------
template <size_t Tdimension>
math::tMatrix < Tdimension + 1, Tdimension + 1 > CreateTransformationMatrix(const math::tMatrix<Tdimension, Tdimension> &rotation, const math::tVector<Tdimension> &translation)
{
  math::tMatrix < Tdimension + 1, Tdimension + 1 > result;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    for (size_t k = 0; k < Tdimension; ++k)
    {
      result[i][k] = rotation[i][k];
    }
    result[i][Tdimension] = translation[i];
  }
  result[Tdimension][Tdimension] = 1;
  return result;
}
}

//----------------------------------------------------------------------
// tIterativeClosestPoint constructors
//----------------------------------------------------------------------
template <size_t Tdimension>
tIterativeClosestPoint<Tdimension>::tIterativeClosestPoint()
{
  this->Reset();
}

template <size_t Tdimension>
template <typename TModelIterator, typename TDataIterator>
tIterativeClosestPoint<Tdimension>::tIterativeClosestPoint(TModelIterator model_begin, TModelIterator model_end, TDataIterator data_begin, TDataIterator data_end,
    double sufficient_improvement_threshold, unsigned int max_iterations) :
  model(model_begin, model_end),
  data(data_begin, data_end)
{
  this->DoICP(sufficient_improvement_threshold, max_iterations);
}

//----------------------------------------------------------------------
// tIterativeClosestPoint Reset
//----------------------------------------------------------------------
template <size_t Tdimension>
void tIterativeClosestPoint<Tdimension>::Reset()
{
  this->correspondence_pairs.clear();
  this->mse = 0;
  this->transformation = decltype(this->transformation)::Identity();
  this->transformed_model.clear();
}

//----------------------------------------------------------------------
// tIterativeClosestPoint SetModel
//----------------------------------------------------------------------
template <size_t Tdimension>
template <typename TIterator>
void tIterativeClosestPoint<Tdimension>::SetModel(TIterator begin, TIterator end)
{
  this->model.assign(begin, end);
}

//----------------------------------------------------------------------
// tIterativeClosestPoint SetData
//----------------------------------------------------------------------
template <size_t Tdimension>
template <typename TIterator>
void tIterativeClosestPoint<Tdimension>::SetData(TIterator begin, TIterator end)
{
  this->data.assign(begin, end);
}

//----------------------------------------------------------------------
// tIterativeClosestPoint DoICP
//----------------------------------------------------------------------
template <size_t Tdimension>
void tIterativeClosestPoint<Tdimension>::DoICP(double sufficient_improvement_threshold, unsigned int max_iterations)
{
  assert(this->model.size() >= 2 && this->data.size() >= 2);
#ifdef RRLIB_MODEL_FITTING_DEBUG_ICP
  static_assert(Tdimension == 2, "Debug visualization of ICP is only supported for 2D");
  geometry::tBoundingBox<2, double> bounding_box(this->model.begin(), this->model.end());
  bounding_box.Add(this->data.begin(), this->data.end());
  auto &window = highgui::tWindow::GetInstance("Debug ICP", bounding_box.Max().X() - bounding_box.Min().X() + 21, bounding_box.Max().Y() - bounding_box.Min().Y() + 21, bounding_box.Min().X() - 10, bounding_box.Min().Y() - 10);

  window.Clear();
  window.SetColor(1);
  for (const auto & p : this->model)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 3, true);
  }
  window.SetColor(0);
  for (auto & p : this->data)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 2, true);
  }
  window.Render();
#endif

  this->Reset();

  auto model = this->model;
  for (unsigned int iteration = 0; iteration < max_iterations; ++iteration)
  {
    std::vector<double> distances;
    this->FindCorrespondencePairs(distances, model);
    this->FilterCorrespondencePairs(distances);

    Visualize(model, this->data, this->correspondence_pairs);

    auto last_mse = this->mse;
    this->mse = CalculateMSE(distances);
    if (std::abs(last_mse - this->mse) < sufficient_improvement_threshold)
    {
      RRLIB_LOG_PRINT(DEBUG, "found solution after ", iteration, " iterations: improvement = ", std::abs(last_mse - this->mse));
      break;
    }

    auto rotation = this->FindRotation(model);
    auto transformation = CreateTransformationMatrix(rotation, this->data.front() - rotation * model.front());
    this->transformation = transformation * this->transformation;

    for (auto & m : model)
    {
      m = transformation.MultiplyHomogeneously(m);
    }

    Visualize(model, this->data, this->correspondence_pairs);
  }

#ifdef RRLIB_MODEL_FITTING_DEBUG_ICP
  window.Clear();
  window.SetColor(1);
  for (const auto & p : this->model)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 2, true);
  }
  window.SetColor(0);
  for (auto & p : this->data)
  {
    window.DrawCircleShifted(p.X(), p.Y(), 3, true);
  }
  window.SetColor(4);
  for (size_t i = 0; i < this->model.size(); ++i)
  {
    const auto &a = this->model[i];
    const auto &b = this->TransformedModel()[i];
    window.DrawLineShifted(a.X(), a.Y(), b.X(), b.Y());
    window.DrawCircleShifted(b.X(), b.Y(), 4, false);
  }
  window.Render();
#endif
}

//----------------------------------------------------------------------
// tIterativeClosestPoint Model
//----------------------------------------------------------------------
template <size_t Tdimension>
const std::vector<math::tVector<Tdimension>> &tIterativeClosestPoint<Tdimension>::Model() const
{
  return this->model;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint Data
//----------------------------------------------------------------------
template <size_t Tdimension>
const std::vector<math::tVector<Tdimension>> &tIterativeClosestPoint<Tdimension>::Data() const
{
  return this->data;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint CorrespondencePairs
//----------------------------------------------------------------------
template <size_t Tdimension>
const std::vector<std::pair<size_t, size_t>> &tIterativeClosestPoint<Tdimension>::CorrespondencePairs() const
{
  return this->correspondence_pairs;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint Error
//----------------------------------------------------------------------
template <size_t Tdimension>
double tIterativeClosestPoint<Tdimension>::Error() const
{
  return this->mse;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint Transformation
//----------------------------------------------------------------------
template <size_t Tdimension>
auto tIterativeClosestPoint<Tdimension>::Transformation() const -> const tTransformation &
{
  return this->transformation;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint TransformedModel
//----------------------------------------------------------------------
template <size_t Tdimension>
const std::vector<math::tVector<Tdimension>> &tIterativeClosestPoint<Tdimension>::TransformedModel() const
{
  if (this->transformed_model.empty())
  {
    this->transformed_model.reserve(this->model.size());
    for (const auto & m : this->model)
    {
      this->transformed_model.emplace_back(this->transformation.MultiplyHomogeneously(m));
    }
  }
  return this->transformed_model;
}

//----------------------------------------------------------------------
// tIterativeClosestPoint FindCorrespondencePairs
//----------------------------------------------------------------------
template <size_t Tdimension>
void tIterativeClosestPoint<Tdimension>::FindCorrespondencePairs(std::vector<double> &distances, const std::vector<tSample> &model)
{
  this->correspondence_pairs.clear();
  distances.clear();
  for (size_t i = 0; i < model.size(); ++i)
  {
    size_t best = 0;
    auto best_distance = (model[i] - this->data[0]).Length();
    for (size_t k = 1; k < this->data.size(); ++k)
    {
      const auto distance = (model[i] - this->data[k]).Length();
      if (distance < best_distance)
      {
        best = k;
        best_distance = distance;
      }
    }
    this->correspondence_pairs.push_back(std::make_pair(i, best));
    distances.push_back(best_distance);
  }
}

//----------------------------------------------------------------------
// tIterativeClosestPoint FilterCorrespondencePairs
//----------------------------------------------------------------------
template <size_t Tdimension>
void tIterativeClosestPoint<Tdimension>::FilterCorrespondencePairs(std::vector<double> &distances)
{
  std::vector<std::pair<size_t, size_t>> filtered_correspondence_pairs;
  std::vector<double> filtered_distances;
  auto sorted_distances = distances;
  std::sort(sorted_distances.begin(), sorted_distances.end());
  const auto median = sorted_distances[sorted_distances.size() / 2];
  for (size_t i = 0; i < distances.size(); ++i)
  {
    if (distances[i] <= Tdimension * median)
    {
      filtered_correspondence_pairs.emplace_back(correspondence_pairs[i]);
      filtered_distances.emplace_back(distances[i]);
    }
  }
  correspondence_pairs = std::move(filtered_correspondence_pairs);
  distances = std::move(filtered_distances);
}

//----------------------------------------------------------------------
// tIterativeClosestPoint FindRotation
//----------------------------------------------------------------------
template <size_t Tdimension>
auto tIterativeClosestPoint<Tdimension>::FindRotation(const std::vector<tSample> &model) -> tRotation
{
  tSample model_cog, data_cog;
  for (const auto & c : this->correspondence_pairs)
  {
    model_cog += model[c.first];
    data_cog += this->data[c.second];
  }
  model_cog *= 1.0 / model.size();
  data_cog *= 1.0 / this->data.size();
  auto translation = data_cog - model_cog;

  using tMatrix = math::tMatrix<Tdimension, Tdimension>;
  tMatrix h;
  for (const auto & c : correspondence_pairs)
  {
    h += tMatrix(model[c.first] + translation, this->data[c.second]);
  }

  tMatrix u, vt;
  cv::Mat w;
  cv::SVD::compute(cv::Mat(Tdimension, Tdimension, CV_64FC1, &h), w, cv::Mat(Tdimension, Tdimension, CV_64FC1, &u), cv::Mat(Tdimension, Tdimension, CV_64FC1, &vt));

  auto correction = tMatrix::Identity();
  correction[Tdimension - 1][Tdimension - 1] = h.Determinant() >= 0 ? 1 : -1;
  auto rotation = vt.Transposed() * correction * u.Transposed();

  return rotation;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
