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
/*!\file    test_ransac_color_fittingl.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-03-19
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>

#include <cv.h>
#include <highgui.h>

#include "rrlib/math/tVector.h"
#include "rrlib/logging/configuration.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/model_fitting/tRansacModel.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;
using namespace rrlib::model_fitting;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

class tColorDetection : public tRansacModel<tVec3d>
{

public:

  tColorDetection(const IplImage *image)
  {
    size_t dimension = image->width * image->height;
    this->Initialize(dimension);
    for (size_t i = 0; i < dimension; ++i)
    {
      tSample sample(reinterpret_cast<double *>(cvGet1D(image, i).val));
      this->AddSample(sample);
    }
    if (!this->DoRANSAC(100, 0.5, 2))
    {
      throw std::runtime_error("Failed to fit RANSAC model during construction!");
    }
  }

  const size_t MinimalSetSize() const
  {
    return 1;
  }

  const tSample &GetModel() const
  {
    return this->model;
  }

private:

  tSample model;

  const bool FitToMinimalSampleIndexSet(const std::vector<size_t> &sample_index_set)
  {
    this->model = this->Samples()[sample_index_set[0]];
    return true;
  }

  const bool FitToSampleIndexSet(const std::vector<size_t> &sample_index_set)
  {
    this->model = tSample();
    for (size_t i = 0; i < sample_index_set.size(); ++i)
    {
      this->model += this->Samples()[sample_index_set[i]];
    }
    this->model /= sample_index_set.size();
    return true;
  }

  const double GetSampleError(const tSample &sample) const
  {
    return (sample - this->model).SquaredLength();
  }

};


int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::SetDomainMaxMessageLevel(".", rrlib::logging::tLogLevel::DEBUG_VERBOSE_3);
  rrlib::logging::SetDomainPrintsLocation(".", false);

  IplImage *image = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
  cvSet(image, cvScalar(128, 128, 128));
  int noise = argc > 1 ? atoi(argv[1]) : 1000;
  for (int i = 0; i < noise; ++i)
  {
    int pixel = rand() % (image->width * image->height);
    unsigned char red  = rand() % 256;
    unsigned char green  = rand() % 256;
    unsigned char blue  = rand() % 256;
    cvSet1D(image, pixel, cvScalar(blue, green, red));
  }
  cvShowImage("Random Image", image);

  tColorDetection color_detection(image);
  tColorDetection::tSample color(color_detection.GetModel());
  std::cout << "Inlier: " << color_detection.NumberOfInliers() << ", Ratio: " << color_detection.InlierRatio() << ", Error: " << color_detection.Error() << std::endl;
  std::cout << "Color: " << tVec3d(color) << std::endl;

  cvWaitKey(0);

  cvReleaseImage(&image);
  cvDestroyAllWindows();

  return EXIT_SUCCESS;
}
