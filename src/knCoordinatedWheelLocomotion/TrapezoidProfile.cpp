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
#include <cstdlib>
#include <exception>
#include <stdexcept>

#include "TrapezoidProfile.h"

namespace kn {

  using namespace std;

  TrapezoidProfile::TrapezoidProfile(const double maxSpeed, const double maxAccel)
    : m_absMaxSpeed(maxSpeed), 
      m_absMaxAccel(maxAccel), 

      m_initialPosition(NAN), 
      m_initialSpeed(NAN),
      m_targetPosition(NAN),

      m_accelToCruise(NAN),
      m_accelToTarget(NAN),
      m_cruiseSpeed(NAN),
      m_cruiseBegTime(NAN),
      m_cruiseBegPosition(NAN),
      m_cruiseEndTime(NAN),
      m_cruiseEndPosition(NAN),
      m_targetTime(NAN)
  {
  }

  void TrapezoidProfile::setLimits(const double maxSpeed, const double maxAccel)
  {
    m_absMaxSpeed = maxSpeed;
    m_absMaxAccel = maxAccel;
  }

  void TrapezoidProfile::initializeState(
    const double initialPosition, const double initialSpeed, const double targetPosition, bool stopAtNodalPt)
  {
    m_initialPosition = initialPosition;
    m_initialSpeed = initialSpeed;
    m_targetPosition = targetPosition;

    // Signed distance to target
    double targetDist = m_targetPosition - m_initialPosition;
    double nodalPtDist = m_initialSpeed * fabs(m_initialSpeed) / 2. / m_absMaxAccel;

    if (stopAtNodalPt) {
      targetDist = nodalPtDist;
      m_targetPosition = targetDist + m_initialPosition;
    }

    // Set Target Speeds and Accelerations
    double cruiseSpdSgn = (targetDist > nodalPtDist) ? 1.0 : -1.0;

    m_cruiseSpeed = cruiseSpdSgn * m_absMaxSpeed;
    m_accelToCruise = (initialSpeed > m_cruiseSpeed) ? -m_absMaxAccel :  m_absMaxAccel;
    m_accelToTarget = -cruiseSpdSgn * m_absMaxAccel;
    
    double nonCruiseDist = (m_cruiseSpeed * m_cruiseSpeed - m_initialSpeed * m_initialSpeed) / 2. / m_accelToCruise
      - m_cruiseSpeed * m_cruiseSpeed / 2. / m_accelToTarget;

    double cruiseTime = (targetDist - nonCruiseDist) / m_cruiseSpeed;  

    if (cruiseTime < 0.) {
      double cruiseSpdSq = m_accelToCruise * targetDist + m_initialSpeed * m_initialSpeed / 2.; 
      if (cruiseSpdSq > 0.)
        m_cruiseSpeed = cruiseSpdSgn * sqrt(cruiseSpdSq);
      else
        m_cruiseSpeed = 0.;
      cruiseTime = 0.;
    }

    m_cruiseBegTime = (m_cruiseSpeed - m_initialSpeed) / m_accelToCruise;
    m_cruiseEndTime = m_cruiseBegTime + cruiseTime;

    m_cruiseBegPosition = m_initialPosition 
      + (m_cruiseSpeed + m_initialSpeed) * m_cruiseBegTime / 2.;
    m_cruiseEndPosition = m_cruiseBegPosition + cruiseTime * m_cruiseSpeed;

    m_targetTime = m_cruiseEndTime - m_cruiseSpeed / m_accelToTarget;

    // Temporary sanity check -- remove this when we are sure this condition never gets hit.
    if (fabs(m_targetPosition - derivedTargetPosition()) >= 1E-4 || isnan(m_targetPosition) || isnan(derivedTargetPosition()))
    {
      cerr << " ***** TRAPEZOID PROFILE SANITY CHECK FAILED *****\n"
           << " TargetPosition: " << m_targetPosition << "\n"
           << " DerivedTargetPosition: " << derivedTargetPosition() << "\n"
           << *this << "\n";
      throw std::logic_error("Trapezoid Profile Sanity Check Failed");
    }
    //    assert(fabs(m_targetPosition - derivedTargetPosition()) < 1E-4);
  }

  void TrapezoidProfile::computeState(const double time, double& position, double& speed) const
  {
    if (time < 0) {
      position = NAN;
      speed = NAN;
    } else if (time <= m_cruiseBegTime) {
      speed = m_initialSpeed + m_accelToCruise * time;
      position = m_initialPosition + (speed + m_initialSpeed) * time / 2.;
    } else if (time <= m_cruiseEndTime) {
      speed = m_cruiseSpeed;
      position = m_cruiseBegPosition + speed * (time - m_cruiseBegTime);
    } else if (time <= m_targetTime) {
      double deltaTime = time - m_cruiseEndTime;
      speed = m_cruiseSpeed + m_accelToTarget * deltaTime;
      position = m_cruiseEndPosition + (speed + m_cruiseSpeed) * deltaTime / 2.;
    } else {
      speed = 0.;
      position = m_targetPosition;
    }
  }

  double TrapezoidProfile::acceleration(const double time) const 
  {
    double acceleration = NAN;

    if (time < 0) {
      acceleration = NAN;
    } else if (time <= m_cruiseBegTime) {
      acceleration = m_accelToCruise;
    } else if (time <= m_cruiseEndTime) {
      acceleration = 0.;
    } else if (time <= m_targetTime) {
      acceleration = m_accelToTarget;
    } else {
      acceleration = 0;
    }

    return acceleration;
  }

  //---------- DEBUG FUNCTIONS ------------------

  std::ostream& operator<<(std::ostream& os, const TrapezoidProfile& tpz) {
    using namespace std;
    os << "------------------TRAPEZOID-PROFILE---------------------------\n"
       << "Max Speed:               " << tpz.m_absMaxSpeed         << std::endl
       << "Max Accel:               " << tpz.m_absMaxAccel         << std::endl
       << "\n"
       << "Initial Position:        " << tpz.m_initialPosition     << std::endl
       << "Initial Speed:           " << tpz.m_initialSpeed        << std::endl 
       << "\n"
       << "AccelToCruise:           " << tpz.m_accelToCruise       << std::endl
       << "\n"
       << "Beg Cruise Time:         " << tpz.m_cruiseBegTime       << std::endl
       << "Beg Cruise Position:     " << tpz.m_cruiseBegPosition   << std::endl
       << "\n"
       << "Cruise Speed:            " << tpz.m_cruiseSpeed         << std::endl
       << "\n"
       << "End Cruise Time:         " << tpz.m_cruiseEndTime       << std::endl
       << "End Cruise Position:     " << tpz.m_cruiseEndPosition   << std::endl
       << "\n"
       << "AccelToTarget:           " << tpz.m_accelToTarget       << std::endl
       << "\n"
       << "End Trajectory Time:     " << tpz.m_targetTime          << std::endl
       << "End Trajectory Position: " << tpz.m_targetPosition      << std::endl
       << "--------------------------------------------------------------\n";
    return os;
  }
}

