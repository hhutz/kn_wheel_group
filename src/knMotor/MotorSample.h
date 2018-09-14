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

#ifndef kn_MotorSample_h
#define kn_MotorSample_h

#include "knMotor_Export.h"
#include "MotorState.h"

#include "ace/Time_Value.h"

#include <iosfwd>

namespace kn
{
  class MotorSample;
  knMotor_Export std::ostream& operator<< (std::ostream& ostr, MotorSample const& rhs);

  //! Sample of the motor state as constructed from telemetry.
  /** This is the base telemetry of an individually controlled motor (Motor interface).
   *
   * This data structure aims at representing the complete abstract
   * state of a motor controller. It probably fails on that attempt.
   */
  class knMotor_Export MotorSample
  {
  public:
    //! The current command status.
    int status;
    //! The timestamp the sample was taken.
    ACE_Time_Value timestamp;
    //! The expected time for reaching the target values in the state.cmd struct.
    /** For cmdProfile commands, this is a rough estimate of the execution time.
     * For cmdTrajectory commands, this is the time, by which the currently
     * targeted trajectory sample will be reached. This is the time given as reference
     * value (ref) when over-writing a trajectory command.
     */
    ACE_Time_Value targetTime;

    //! The motor state.
    MotorState state;
    //! Current reading of the motor (amp).
    /** NAN if not available. */
    double current;
    //! Temperature reading of the motor (celsius).
    /** NAN if not available. */
    double temperature;

    // Are there other motor parameters available/relevant?

    //! Default constructor.
    MotorSample() {}
    //! Initializing constructor.
    MotorSample(int s1,
                ACE_Time_Value const& stamp, ACE_Time_Value const& target,
                MotorState const& s2, double t, double c) :
      status(s1),
      timestamp(stamp),
      targetTime(target),
      state(s2),
      current(c),
      temperature(t)
    {
    }
  };
}
#endif // kn_MotorSample_h
