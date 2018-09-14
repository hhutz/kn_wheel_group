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

#ifndef kn_MotorProfile_h
#define kn_MotorProfile_h

#include "knMotor_Export.h"

#include <vector>
#include <iosfwd>

namespace kn
{

  class MotorProfile;
  //! Vector of motor profiles.
  typedef std::vector<MotorProfile> MotorProfileVector;

  knMotor_Export std::ostream& operator<< (std::ostream& ostr, MotorProfile const& rhs);
  knMotor_Export std::ostream& operator<< (std::ostream& ostr, MotorProfileVector const& rhs);

  //! Class representing a motor command profile.
  /** The motor command profile denotes the current command
   * executed by the motor.
   */
  class knMotor_Export MotorProfile
  {
  public:
    //! The set of available control modes.
    /** Not all motors support all control modes. */
    enum CtrlMode { CTRL_POSITION, CTRL_TRAPECOID, CTRL_TRAJECTORY, CTRL_HOLD, CTRL_OFF };
    //! Target positions can be specified absolute or relative.
    enum PositionMode { POS_ABS, POS_REL, POS_NA };

    //! Commanded control mode.
    CtrlMode ctrlMode;
    //! Positioning mode for CTRL_POSITION, CTRL_TRAPECOID, CTRL_TRAJECTORY modes.
    /** Control modes, which don't specify position are supposed to set posMode to POS_NA. */
    PositionMode posMode;

    //! Target position.
    /** Control modes, which don't specify position are supposed to set this field to NAN. */
    double position;
    //! Target or max speed.
    /** Control modes, which don't specify speed are supposed to set this field to NAN. */
    double speed;
    //! Max acceleration.
    /**
     * This is an absolute value. It is assumed that the maximum decelleration is -acc.
     * Control modes, which don't specify acceleration are supposed to set this field to NAN.
     */
    double acc;

    //! Default contructor.
    MotorProfile() {}
    //! Initializing constructor.
    /** Note that no consitency checking is done for fields. */
    MotorProfile(CtrlMode cM, PositionMode pM, double p, double s, double a) :
        ctrlMode(cM),
        posMode(pM),
        position(p),
        speed(s),
        acc(a) {}
  };

}
#endif // kn_MotorProfile_h
