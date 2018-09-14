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

#ifndef kn_HALF_TRAP_PROFILE_H
#define kn_HALF_TRAP_PROFILE_H

#include <ostream>
#include <iomanip>

namespace kn
{
  /**
   * This class implements a Continuous Half Trapezoid Speed profile.  The
   * profile calculations assumes an initial starting speed and position,
   * a constant acceleration or deceleration to a cruise speed,
   * followed by cruise segment. */
  class HalfTrapProfile
  {
  public:
    HalfTrapProfile(const double maxSpeed = NAN, const double maxAccel = NAN);

    void setLimits(const double maxSpeed, const double maxAccel);

    void initializeState(
      const double initialPosition, const double initialSpeed);

    void computeState(const double time, double& position, double& speed) const;

    double acceleration(const double time) const;

  private:
    double m_cruiseSpeed;
    double m_absMaxAccel;

    double m_initialPosition;
    double m_initialSpeed;

    double m_accelToCruise;
    double m_cruiseBegTime;
    double m_cruiseBegPosition;

    friend std::ostream& operator<<(std::ostream& os, const HalfTrapProfile& htp);
  };
}

#endif // kn_HALF_TRAP_PROFILE_H
