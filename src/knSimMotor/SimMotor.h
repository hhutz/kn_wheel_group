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

#ifndef kn_SimMotor_h
#define kn_SimMotor_h

#include "knSimMotor_Export.h"

#include "knMotor/MotorProfile.h"
#include "knMotor/MotorSample.h"
#include "knMotorImpl/MotorImpl.h"

namespace kn
{
  class knSimMotor_Export SimMotor : public MotorImpl
  {
  public:
    static unsigned int const TARGET_LOOKAHEAD;;

    SimMotor(MotorParameters const * params);
    virtual ~SimMotor() throw();

    // Motor base interface

    virtual int tryRecover();
    virtual int stop(StopMode mode);

    // Motor interface

    virtual int cmdProfile(MotorProfile const& profile);
    virtual int cmdTrajectory(int ref,
                              double frequency,
                              DoubleVector const& postions,
                              DoubleVector const& speeds);

    // Simulation update
    virtual void executeTimeSlice(double frequency);
    virtual void setTime(ACE_Time_Value const& now);
    void startTrajectory(double frequency);

    static void pushSamplesToQueue(DoubleVector& feed, DoubleVector& queue, unsigned int num);

    // MotorHelper interface
    virtual bool calibrateAsync() const throw();
    virtual int calibrateHook();
    virtual void onCommandCompletion() throw();

  protected:
    void trapezoidCtrl(double slice);
    void trajectoryCtrl(double slice);
    void velocityCtrl(double slice);
    void pushTrajectoryPoints(unsigned int num);

    ACE_Recursive_Thread_Mutex m_mutex;
    ACE_Condition_Recursive_Thread_Mutex m_condition;

    ACE_Time_Value m_now;
    DoubleVector m_positionQueue;
    DoubleVector m_speedQueue;
  };
}
#endif // kn_SimMotor_h
