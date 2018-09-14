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

#ifndef kn_FastestSteerAndDriveCWL_h
#define kn_FastestSteerAndDriveCWL_h

#include "CoordinatedWheelLocomotion.h"
#include "TrapezoidProfile.h"
#include "HalfTrapProfile.h"

#include <iosfwd>

namespace kn
{
  class knCoordinatedWheelLocomotion_Export FastestSteerAndDriveCWL : public CoordinatedWheelLocomotion
  {
  public:

    /**
       This locomotion class generates rover wheel commands, such that the rover achieves a target curvature and wheel
       speed as fast as possible, up to the hardware limits of motor steer speed, motor steer acceleration, motor
       drive speed, motor drive acceleration.  This locomotion type adheres to the general CoordinatedWheelLocomotion
       properties (see CoordinatedHWeelLocomotion.h), has the following specific properties:

       1) The initial condition for each path consists of the following:
       a) Curvature
       b) Wheel Index and Speed of fastest wheel
       c) Wheel Index and Steer Rate of fastest steering wheel
       d) Position of all steer motors

       2) The target condition for each path consists of the following:
       a) Curvature
       b) Speed of fastest wheel
       c) Timeout value, or NaN to stop path when target achieved

       3) The method for getting from the initial condition to the target is as follows:

       a) While not at the target curvature, continuously adjust the curvature as follows:

       i) While all steer wheels are rotating at less than the maximum hardware steer rate, use a steer
       acceleration which causes at least one wheel to hit the maximum steer acceleration, and no wheels to
       exceed the maximum steer acceleration.

       ii) Once the maximum steer rate is hit, continuously adjust the curvature such that at least one wheel
       is steering at the maximum steer rate, and no wheels are steering faster.

       iii) After a calculated time, decelerate the steer speed such that the target curvature is reached
       precisely when the steer speed is zero, and at least one wheel is decelerating at the hardware maximum,
       and no wheel is decelerating faster.

       b) Continuously adjust the rover speed such that:

       i) While no wheel is driving at the maximum wheel speed, at least one wheel should accelerate as fast
       as possible, but no wheel should accelerate faster than the maximum wheel acceleration.

       ii) While at least one wheel is driving at the maximum wheel speed, maintain at least one wheel at this
       maximum speed, without any other wheel exceeding this speed.

       c) Once the timeout is reached, immediately slow down and straighten wheels, by using the same method as when 
       reaching a target curvature and speed, but with a target curvature of zero and speed of zero.
    */

    FastestSteerAndDriveCWL(double sampleFrequency, double wheelRadius, Vector2Vector const& wheelLocations);

    // set variables used to achieve targets
    void setTransition(double wheelAcceleration, double targetSteerAcceleration, double targetSteerRate) throw();

    unsigned int appendAlignmentPath(double pathTime);
    // parameters are target variables which are not used to achieve other targets 
    unsigned int appendPath(double pathTime, double targetWheelSpeed, double targetCurvature, double targetCrab = 0.0);

    // DEBUG Functions
    bool printWarnings(std::ostream& ostr, bool printDebug);
  
    
  protected:
    void getFastestSteerWheelIndex(const bool& rightTurn, unsigned int& fastestSteerWheelIndex, 
                                   unsigned int& fastestDriveWheelIndex, double& driveSpeedRatio) const;
    unsigned int maxPoints(double pathTime);

    // Keep track of the fastest wheel speed which the rover should not exceed.
    void resetMaxDriveSpeed();
    void setMaxDriveSpeed(double speed);

  private:
    double m_wheelAcceleration;       //!< m/(s*s)
    double m_targetSteerAcceleration; //!< rad/(s*s)
    double m_targetSteerRate;         //!< rad/s

    double m_maxDriveSpeed;

    TrapezoidProfile m_steerProfile;
#ifdef ENABLE_FSD_CRABBING
    TrapezoidProfile m_crabProfile;
#endif
    HalfTrapProfile  m_driveProfile;

    std::vector<TrapezoidProfile> m_alignmentPath;
  };
}

#endif // kn_FastestSteerAndDriveCWL_h
