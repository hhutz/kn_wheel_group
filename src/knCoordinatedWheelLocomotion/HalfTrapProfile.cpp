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

#include <cmath>
#include <cstring>
#include <vector>
#include <iostream>
#include <cassert>

#include "HalfTrapProfile.h"

namespace kn {

  using namespace std;

  HalfTrapProfile::HalfTrapProfile(const double maxSpeed, const double maxAccel)
    : m_cruiseSpeed(maxSpeed), 
      m_absMaxAccel(maxAccel), 

      m_initialPosition(NAN), 
      m_initialSpeed(NAN),

      m_accelToCruise(NAN),
      m_cruiseBegTime(NAN),
      m_cruiseBegPosition(NAN)
  {
  }

  void HalfTrapProfile::setLimits(const double maxSpeed, const double maxAccel)
  {
    m_cruiseSpeed = maxSpeed;
    m_absMaxAccel = maxAccel;
  }

  void HalfTrapProfile::initializeState(
    const double initialPosition, const double initialSpeed) 
  {
    m_initialPosition = initialPosition;
    m_initialSpeed = initialSpeed;

    // Set Target Speeds and Accelerations
    m_accelToCruise = (initialSpeed > m_cruiseSpeed) ? -m_absMaxAccel :  m_absMaxAccel;
    
    m_cruiseBegTime = (m_cruiseSpeed - m_initialSpeed) / m_accelToCruise;
    
    m_cruiseBegPosition = m_initialPosition 
      + (m_cruiseSpeed + m_initialSpeed) * m_cruiseBegTime / 2.;
  }

  void HalfTrapProfile::computeState(const double time, double& position, double& speed) const
  {
    if (time < 0) {
      position = NAN;
      speed = NAN;
    } else if (time <= m_cruiseBegTime) {
      speed = m_initialSpeed + m_accelToCruise * time;
      position = m_initialPosition + (speed + m_initialSpeed) * time / 2.;
    } else {
      speed = m_cruiseSpeed;
      position = m_cruiseBegPosition + speed * (time - m_cruiseBegTime);
    } 
  }

  double HalfTrapProfile::acceleration(const double time) const
  {
    double acceleration = NAN;

    if (time < 0) {
      acceleration = NAN;
    } else if (time <= m_cruiseBegTime) {
      acceleration = m_accelToCruise;
    } else {
      acceleration = 0;
    } 

    return acceleration;
  }

  //---------- DEBUG FUNCTIONS ------------------
  
  std::ostream& operator<<(std::ostream& os, const HalfTrapProfile& htp) {
    using namespace std;
    os << "-------------HALF-TRAPEZOID-PROFILE---------------------------\n"
       << "Max Accel:               " << htp.m_absMaxAccel         << std::endl
       << "\n"
       << "Initial Position:        " << htp.m_initialPosition     << std::endl
       << "Initial Speed:           " << htp.m_initialSpeed        << std::endl 
       << "\n"
       << "AccelToCruise:           " << htp.m_accelToCruise       << std::endl
       << "\n"
       << "Beg Cruise Time:         " << htp.m_cruiseBegTime       << std::endl
       << "Beg Cruise Position:     " << htp.m_cruiseBegPosition   << std::endl
       << "\n"
       << "Cruise Speed:            " << htp.m_cruiseSpeed         << std::endl
       << "--------------------------------------------------------------\n";
    return os;
  }
}

