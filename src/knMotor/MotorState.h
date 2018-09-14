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

#ifndef kn_MotorState_h
#define kn_MotorState_h

#include "knMotor_Export.h"
#include "MotorProfile.h"

#include <vector>
#include <iosfwd>

namespace kn
{
  class MotorState;
  std::ostream& operator<< (std::ostream& ostr, MotorState const& rhs);

  //! Motor state as constructed from telemetry.
  /** This data structure aims at representing the complete abstract
   * state of a motor controller. It probably fails on that attempt.
   */
  class knMotor_Export MotorState
  {
  public:
    //! The current command, as executed by the motor.
    /** The motor profile specifies the target position as well
     * as the maximum speed and acceleration values.
     */
    MotorProfile cmd;

    //! Current postition of the motor (in rad).
    double position;
    //! Current speed of the motor (in rad/s).
    double speed;

    //! Default constructor.
    MotorState() {}
    //! Initializing constructor.
    MotorState(MotorProfile const& cmd, double p, double s) :
        cmd(cmd),
        position(p),
        speed(s) {}
  };

  typedef std::vector<MotorState> MotorStateVector;
}
#endif // kn_MotorState_h
