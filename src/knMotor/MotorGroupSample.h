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

#ifndef kn_MotorGroupSample_h
#define kn_MotorGroupSample_h

#include "MotorState.h"
#include "knMotor_Export.h"

#include "ace/Time_Value.h"

#include <iosfwd>

namespace kn
{
  class MotorGroupSample;
  knMotor_Export std::ostream& operator<< (std::ostream& ostr, MotorGroupSample const& rhs);

  //! Sample of the motor group state as constructed from telemetry.
  /** This is the base telemetry of a group of synchronisly controlled motor (MotorGroup interface).
   *
   * This data structure aims at representing the complete abstract
   * state of a motor group. It probably fails on that attempt.
   */
  class knMotor_Export MotorGroupSample
  {
  public:
     //! Vector of integers.
    typedef std::vector<int> IntVector;
    //! Vector of doubles.
    typedef std::vector<double> DoubleVector;

    //! The cmdsStatus status of the motor group.
    /** 0 is okay, negative numbers represent errors,
     * positive numbers are for nominal, but special states
     * (such as calibration etc). See KNMOTOR_xyz constants.
     */
    int status;
    //! Timestamp of the telemetry report.
    ACE_Time_Value timestamp;
    //! The expected time for reaching the target values in the motors[].state.cmd structs.
    /** For cmdProfile commands, this is a rough estimate of the execution time.
     * For cmdTrajectory commands, this is the time, by which the currently
     * targeted trajectory sample will be reached. This is the time given as reference
     * value (ref) when over-writing a trajectory command. */
    ACE_Time_Value targetTime;

    //! State of the individual motors of the group.
    MotorStateVector motors;
    //! The status of the individual motors.
    /** For regular status, this should be the same as status.
     * Errors can be differentiated by motor.
     * If empty, status is assumed identical for all motors.
     */
    IntVector motorStatus;

    //! Current readings of the individual motors (amp).
    /** Empty if not available. */
    DoubleVector currents;
    //! Temperature readings of the individual motors (celsius).
    /** Empty if not available. */
    DoubleVector temperatures;

    // Are there other motor-group parameters available/relevant?
  };
}

#endif // kn_MotorGroupSample_h
