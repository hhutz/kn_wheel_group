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

#ifndef kn_MotorFuture_h
#define kn_MotorFuture_h

#include "MotorState.h"

#include "ace/Time_Value.h"

namespace kn
{
  //! Sample of motor state as expected at a future time.
  /** This is the base telemetry of an individually controlled motor (Motor interface).
   *
   * This data structure aims at representing the complete abstract
   * state of a motor controller. It probably fails on that attempt.
   */
  class MotorFuture
  {
  public:
    //! The time of the expected future state.
    ACE_Time_Value targetTime;

    double position;
    double speed;

    //! Default constructor.
    MotorFuture();
    //! Initializing constructor.
    MotorFuture(ACE_Time_Value const& target,
                double p,
                double s) :
      targetTime(target),
      position(p),
      speed(s) {}
  };
}
#endif // kn_MotorFuture_h
