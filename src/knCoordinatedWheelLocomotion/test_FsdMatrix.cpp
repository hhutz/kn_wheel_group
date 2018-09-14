/* -*- C++ -*- *****************************************************************
 * Copyright (c) 2013 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * Licensed under the NASA Open Source Agreement, Version 1.3 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   https://opensource.org/licenses/NASA-1.3
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************
 *
 * Project: RoverSw
 * Module: FSD Locomotion
 * Author: Xavier Bouyssounouse
 *
 *****************************************************************************/

// This file generates matlab plots to help validate FSD locomotion.
//
// Usage: testFsdMatrix     <-- Generates Matrix of FSD plot data.
// Usage: testFsdMatrix reg <-- Generates Regression Data instead of plot data.
//////////////////////////////////////////////////////////////////////////////

#include "FsdMatlabPlots.h"
#include <string>
#include <boost/filesystem/operations.hpp>


int main(int argc, char **argv) 
{
  double trajTime = 10.0;
  FsdMatlabPlots plots(argc == 1);

  const unsigned int numTimes = 3;
  const double times[3] = {2.5, 5.0, 1.0};

  const unsigned int numSpeeds = 3;
  const double speeds[3]  = {0, +.5, -.5};

  const unsigned int numCurvatures = 3;
  const double curvatures[3] = {0, +2, -2};

  const unsigned int numCrabs = 3;
  const double crabs[3] = {0, +.5, -.5};

  for (unsigned int timeIdx = 0; timeIdx < numTimes; ++timeIdx)
    for (unsigned int initSpeedIdx = 0; initSpeedIdx < numSpeeds; ++initSpeedIdx)
      for (unsigned int finalSpeedIdx = 0; finalSpeedIdx < numSpeeds; ++finalSpeedIdx)
        for (unsigned int initCurvatureIdx = 0; initCurvatureIdx < numCurvatures; ++initCurvatureIdx)
          for (unsigned int finalCurvatureIdx = 0; finalCurvatureIdx < numCurvatures; ++finalCurvatureIdx)
            for (unsigned int initCrabIdx = 0; initCrabIdx < numCrabs; ++initCrabIdx)
              for (unsigned int finalCrabIdx = 0; finalCrabIdx < numCrabs; ++finalCrabIdx)
              {
                // Initialize to 1 m/s2 wheel accel, and 1 deg/sec steer rate, 1 deg/s^2 steer accel.
                plots.initPath(
                  0.5,                                // maxWheelSpeed
                  1.0,                                // wheelAccel
                  1.0,                                // steerRate
                  1.0);                               // steerAccel 
                
                plots.computePath(
                  times[timeIdx],                     // path segment time 
                  speeds[initSpeedIdx],               // target speed
                  curvatures[initCurvatureIdx],       // target curvature
                  crabs[initCrabIdx]);                // target crab
                
                plots.appendPath(
                  trajTime - times[timeIdx],               // path segment time 
                  speeds[finalSpeedIdx],               // target speed
                  curvatures[finalCurvatureIdx],       // target curvature
                  crabs[finalCrabIdx]);                // target crab

                plots.appendPath(NAN,0,0,0);
                
                std::ostringstream suffix;
                
                suffix << timeIdx
                       << "_" << initSpeedIdx << finalSpeedIdx
                       << "_" << initCurvatureIdx << finalCurvatureIdx
                       << "_" << initCrabIdx << finalCrabIdx;

                if (!boost::filesystem::exists(std::string("path_") + suffix.str() + ".pdf"))
                {
                  plots.outputMatlab("path", suffix.str());
                }
              }
  return 0;
}

