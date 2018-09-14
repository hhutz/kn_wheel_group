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

#ifndef kn_TRAPEZOID_PROFILE_H
#define kn_TRAPEZOID_PROFILE_H

#include <ostream>
#include <iomanip>

namespace kn
{
  /**
   * This class implements a Continuous Trapezoid Speed profile.  The
   * profile calculations assumes an initial starting speed and position,
   * a constant acceleration or deceleration to a cruise speed, a
   * constant cruise speed segment, and a constant acceleration or
   * deceleration to zero speed and a given position.  The speed
   * profiles may cross zero speed, and also include the non-trapezoid
   * profiles of descent-cruise-descent and climb-cruise-climb.  This
   * logic automatically handles overspeeds and overshoots.
   *
   *   * Parameters are calculated during initialization, as follows:
   *      - Calculate the constant speed "cruise" section length.
   *      - If less than zero, then reduce the cruise speed to match
   *        available distance.
   *
   *   * Once initialized, the speed and distance can be quickly
   *     calculated from the trapezoid parameters.
   */
  class TrapezoidProfile
  {
  public:
    TrapezoidProfile(const double maxSpeed = NAN, const double maxAccel = NAN);

    void setLimits(const double maxSpeed, const double maxAccel);

    void initializeState(
      const double initialPosition, const double initialSpeed, const double targetPosition, bool stopAtNodalPt = false);

    void computeState(const double time, double& position, double& speed) const;

    double trajectoryTime() const {
      return m_targetTime;
    }

    double acceleration(const double time) const;

    // DEBUG
    double derivedTargetPosition() const {
      return m_cruiseEndPosition + (m_targetTime - m_cruiseEndTime) * m_cruiseSpeed / 2.;
    }

  private:
    double m_absMaxSpeed;
    double m_absMaxAccel;

    double m_initialPosition;
    double m_initialSpeed;
    double m_targetPosition;

    double m_accelToCruise;
    double m_accelToTarget;
    double m_cruiseSpeed;
    double m_cruiseBegTime;
    double m_cruiseBegPosition;
    double m_cruiseEndTime;
    double m_cruiseEndPosition;
    double m_targetTime;

    friend std::ostream& operator<<(std::ostream& os, const TrapezoidProfile& tpz);
  };
}

#endif // kn_TRAPEZOID_PROFILE_H
