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
 * Module: knMotor
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_WheelGroupSample_h
#define kn_WheelGroupSample_h

#include "MotorGroupSample.h"
#include "knMotor_Export.h"

namespace kn
{
  class WheelGroupSample;
  knMotor_Export std::ostream& operator<< (std::ostream& ostr, WheelGroupSample const& rhs);

  //! Sample of the wheel group state as constructed from telemetry.
  /** This is the base telemetry of a group of synchronisly controlled wheels (WheelGroup interface).
   *
   * This data structure aims at representing the complete abstract
   * state of a wheel group. It probably fails on that attempt.
   */
  class knMotor_Export WheelGroupSample : public MotorGroupSample
  {
  public:
    //! Curvature described by the wheel-group.
    /** fabs(curvature) >= 1000 is considered a point-turn.
     * NAN value describes unaligned wheels or unkown state.
     */
    double curvature;
    //! Curvature rate of the wheel-group.
    /** NAN value describes unaligned wheels or unkown state.
     */
    double curvatureRate;
    //! Velocity of the wheel-group along the curvature.
    /** Unit is m/s.
     * For fabs(curvature) >= 1000 this value is undefined.
     */
    double speed;
    //! Crab-angle described by the wheel-group.
    /** Angle is in radians.
     * NAN value describes unaligned wheels or unkown state.
     */
    double crabAngle;

    //! Crab Rate in rad/s
    double crabRate;

    //! Target curvature in rad.
    /** Note: fabs(curvature) >= 1000 is considered a point-turn. */
    double targetCurvature;
    //! Target curvature rate ins rad/s.
    /** Unit: rad/s. */
    double targetCurvatureRate;
    //! Target velocity along the curvature in m/s.
    double targetSpeed;
    double targetCrabAngle;
    double targetCrabRate;
  };
}
#endif // kn_WheelGroupSample_h
