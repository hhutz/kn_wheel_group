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
 * Module: knMotorImpl
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_MotorImpl_h
#define kn_MotorImpl_h

#include "knMotorImpl_Export.h"
#include "MotorHelper.h"

#include "knMotor/MotorProfile.h"
#include "knMotor/MotorSample.h"
#include "knMotor/Motor.h"

#include "ace/Recursive_Thread_Mutex.h"
#include "ace/Condition_Recursive_Thread_Mutex.h"

namespace kn
{
  class knMotorImpl_Export MotorImpl : virtual public Motor,
                                       public MotorHelper
  {
  public:
    //! Ctor
    /** The motor parameters can be passed on construction,
     * or explicitly set later. Parameters are considered constant though.
     * The class does not take ownership of the parameters.
     */
    MotorImpl(ACE_Recursive_Thread_Mutex& mutex,
	      ACE_Condition_Recursive_Thread_Mutex& condition,
	      MotorParameters const * params = NULL);
    virtual ~MotorImpl() throw();

    // Motor base interface

    virtual MotorParameters const * params() const throw();
    virtual void setParams(MotorParameters const * params) throw();

    virtual int calibrate(CalibrationMode mode);
    virtual int cmdStatus(bool waitForCompletion = false, ACE_Time_Value const * timeout = NULL);

    // Motor interface

    virtual MotorSample motorState(bool waitNextSample = false, ACE_Time_Value const * timeout = NULL);
    virtual int futureMotorState(MotorFuture& state, ACE_Time_Value const& targetTime);

    // MotorHelper interface
    virtual void integrateData(MotorSample const& sample) throw();

  protected:
    ACE_Recursive_Thread_Mutex& m_mutex;
    ACE_Condition_Recursive_Thread_Mutex& m_condition;
  };
}
#endif // kn_MotorImpl_h
