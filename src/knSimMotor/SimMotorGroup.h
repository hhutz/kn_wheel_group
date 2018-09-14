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
 * Module: knSimMotor
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_SimMotorGroup_h
#define kn_SimMotorGroup_h

#include "knSimMotor_Export.h"
#include "SimMotorGroupBase.h"

#include "ace/Recursive_Thread_Mutex.h"
#include "ace/Condition_Recursive_Thread_Mutex.h"

namespace kn
{
  class SimMotor;

  //! Abstract motor group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knSimMotor_Export SimMotorGroup : public virtual SimMotorGroupBase
  {
  public:
    SimMotorGroup(MotorVector const& motors,
                  MotorGroupParameters const * params);
    virtual ~SimMotorGroup() throw();

    // Motor Base interface

    virtual int stop(StopMode mode);

    // Motor interface

    virtual int cmdProfiles(MotorProfileVector const& profiles);
    virtual int cmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& postions,
                                TrajectoryVector const& speeds);

  };
}
#endif // kn_SimMotorGroup_h
