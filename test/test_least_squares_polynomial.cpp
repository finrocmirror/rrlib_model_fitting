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
/*!\file    test_least_squares_polynomial.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-02
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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/tLeastSquaresPolynomial.h"
#include "rrlib/model_fitting/tRansacLeastSquaresPolynomial.h"

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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const unsigned int cDRAW_POLYNOMIAL_STEPS = 50;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

template <size_t Tdegree>
void DrawPolynomial(tWindow &window, const tPolynomial<Tdegree> polynomial)
{
  double x = 0;
  double y = polynomial(x);
  for (size_t i = 1; i < cDRAW_POLYNOMIAL_STEPS + 1; ++i)
  {
    double new_x = x + 1.0 / cDRAW_POLYNOMIAL_STEPS;
    double new_y = polynomial(new_x);
    window.DrawLineNormalized(x, y, new_x, new_y);
    x = new_x;
    y = new_y;
  }
}

int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainConfiguresSubTree(".", true);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainMaxMessageLevel(".", rrlib::logging::eLL_DEBUG_VERBOSE_3);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainPrintsLocation(".", false);

  tWindow &window = tWindow::GetInstance("Least Squares Polynomial Tests", 500, 500);

  std::vector<tVec2d> data;
  data.push_back(tVec2d(0.27, 0.38));
  data.push_back(tVec2d(0.3, 0.42));
  data.push_back(tVec2d(0.1, 0.27));
  data.push_back(tVec2d(0.15, 0.3));
  data.push_back(tVec2d(0.85, 0.17));
  data.push_back(tVec2d(0.2, 0.35));
  data.push_back(tVec2d(0.4, 0.42));
  data.push_back(tVec2d(0.6, 0.26));
  data.push_back(tVec2d(0.65, 0.20));
  data.push_back(tVec2d(0.9, 0.25));
  data.push_back(tVec2d(0.7, 0.11));
  data.push_back(tVec2d(0.45, 0.38));
  data.push_back(tVec2d(0.5, 0.45));
  data.push_back(tVec2d(0.75, 0.12));
  data.push_back(tVec2d(0.55, 0.32));
  data.push_back(tVec2d(0.8, 0.14));
  data.push_back(tVec2d(0.35, 0.43));

//  data.push_back(tVec2f(0.1,  0.2));
//  data.push_back(tVec2f(0.15, 0.3));
//  data.push_back(tVec2f(0.2,  0.25));
//  data.push_back(tVec2f(0.25, 0.5));
//  data.push_back(tVec2f(0.3,  0.23));
//  data.push_back(tVec2f(0.35, 0.35));
//  data.push_back(tVec2f(0.4,  0.34));
//  data.push_back(tVec2f(0.45, 0.3));
//  data.push_back(tVec2f(0.5,  0.45));
//  data.push_back(tVec2f(0.55, 0.5));
//  data.push_back(tVec2f(0.6,  0.48));
//  data.push_back(tVec2f(0.65, 0.51));
//  data.push_back(tVec2f(0.7,  0.45));
//  data.push_back(tVec2f(0.75, 0.42));
//  data.push_back(tVec2f(0.8,  0.4));
//  data.push_back(tVec2f(0.85, 0.32));
//  data.push_back(tVec2f(0.9,  0.31));




  std::cout << "=== Data points ===" << std::endl;

  for (std::vector<tVec2d>::iterator it = data.begin(); it != data.end(); ++it)
  {
    window.DrawCircleNormalized(it->X(), it->Y(), 0.005, true);
  }
  window.Render();

  std::cout << "=== Least squares polynomial of degree 1 ===" << std::endl;

  window.SetColor(1);
  DrawPolynomial(window, tLeastSquaresPolynomial<1>(data.begin(), data.end()));
  window.Render();

  std::cout << "=== Least squares polynomial of degree 2 ===" << std::endl;

  window.SetColor(2);
  DrawPolynomial(window, tLeastSquaresPolynomial<2>(data.begin(), data.end()));
  window.Render();

  std::cout << "=== Least squares polynomial of degree 3 ===" << std::endl;

  window.SetColor(3);
  DrawPolynomial(window, tLeastSquaresPolynomial<3>(data.begin(), data.end()));
  window.Render();

  std::cout << "=== Least squares polynomial of degree 4 ===" << std::endl;

  window.SetColor(4);
  DrawPolynomial(window, tLeastSquaresPolynomial<4>(data.begin(), data.end()));
  window.Render();



  unsigned int max_iterations = 200;
  double satisfactory_support_ratio = 0.8;
  double max_error = 0.05;


  srand(time(0));

  std::cout << "=== Ransac least squares polynomial of degree 1 ===" << std::endl;
  {
    const size_t cDEGREE = 1;
    tRansacLeastSquaresPolynomial<cDEGREE> curve(data.begin(), data.end(), max_iterations, satisfactory_support_ratio, max_error);
    window.Clear();
    for (size_t i = 0; i < curve.Samples().size(); ++i)
    {
      window.SetColor(curve.Assignments()[i] ? cDEGREE : 0);
      window.DrawCircleNormalized(curve.Samples()[i].X(), curve.Samples()[i].Y(), 0.005, true);
    }
    window.SetColor(cDEGREE);
    DrawPolynomial(window, curve);
    window.Render();
  }

  std::cout << "=== Ransac least squares polynomial of degree 2 ===" << std::endl;
  {
    const size_t cDEGREE = 2;
    tRansacLeastSquaresPolynomial<cDEGREE> curve(data.begin(), data.end(), max_iterations, satisfactory_support_ratio, max_error);
    window.Clear();
    for (size_t i = 0; i < curve.Samples().size(); ++i)
    {
      window.SetColor(curve.Assignments()[i] ? cDEGREE : 0);
      window.DrawCircleNormalized(curve.Samples()[i].X(), curve.Samples()[i].Y(), 0.005, true);
    }
    window.SetColor(cDEGREE);
    DrawPolynomial(window, curve);
    window.Render();
  }

  std::cout << "=== Ransac least squares polynomial of degree 3 (50 rounds) ===" << std::endl;
  {
    const size_t cDEGREE = 3;
    tRansacLeastSquaresPolynomial<cDEGREE> curve(data.begin(), data.end(), max_iterations, satisfactory_support_ratio, max_error);
    for (size_t i = 0; i < curve.Samples().size(); ++i)
    {
      window.SetColor(curve.Assignments()[i] ? cDEGREE : 0);
      window.DrawCircleNormalized(curve.Samples()[i].X(), curve.Samples()[i].Y(), 0.005, true);
    }
    window.SetColor(cDEGREE);
    DrawPolynomial(window, curve);
    window.Render();
  }

  std::cout << "=== Ransac least squares polynomial of degree 4 ===" << std::endl;
  {
    const size_t cDEGREE = 4;
    tRansacLeastSquaresPolynomial<cDEGREE> curve(data.begin(), data.end(), max_iterations, satisfactory_support_ratio, max_error);
    window.Clear();
    for (size_t i = 0; i < curve.Samples().size(); ++i)
    {
      window.SetColor(curve.Assignments()[i] ? cDEGREE : 0);
      window.DrawCircleNormalized(curve.Samples()[i].X(), curve.Samples()[i].Y(), 0.005, true);
    }
    window.SetColor(cDEGREE);
    DrawPolynomial(window, curve);
    window.Render();
  }





  window.Render();
  std::cout << "OK" << std::endl;

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
