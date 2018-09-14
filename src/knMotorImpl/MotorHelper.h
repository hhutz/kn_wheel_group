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

#ifndef kn_MotorHelper_h
#define kn_MotorHelper_h

#include "knMotorImpl_Export.h"

#include "knMotor/MotorProfile.h"
#include "knMotor/MotorSample.h"
#include "knMotor/MotorBase.h"

#include <ace/Task.h>

namespace kn
{
  class MotorParameters;
  class MotorFuture;

  class knMotorImpl_Export MotorHelper : ACE_Task_Base
  {
  public:
    //! Ctor
    /** The motor parameters can be passed on construction,
     * or explicitly set later. Parameters are considered constant though.
     * The class does not take ownership of the parameters.
     */
    MotorHelper(MotorParameters const * params = NULL);
    virtual ~MotorHelper() throw();

    virtual void integrateData(MotorSample const& sample) throw();

    int calibrateSkel(MotorBase::CalibrationMode mode);
    int collectFutureMotorState(MotorFuture& state, ACE_Time_Value const& targetTime);
  
    virtual void onCommandCompletion() throw();

    void setStateStop(MotorBase::StopMode mode, ACE_Time_Value const& now) throw();
    int setStateCmdProfile(MotorProfile const& profile, ACE_Time_Value const& now) throw();
    int setStateCmdTrajectory(int ref,
                              double frequency,
                              MotorBase::DoubleVector const& postions,
                              MotorBase::DoubleVector const& speeds);
    void consumeTrajectoryPoints(unsigned int num, 
                                 ACE_Time_Value const& now,
                                 ACE_Time_Value const& targetTime = ACE_Time_Value::zero) throw();
    

    static void consumeSamples(unsigned int newSize, MotorBase::DoubleVector& samples, double& target) throw();

    int indexOffset() const throw() { return m_indexOffset; }

    MotorSample const& sample() const throw() { return m_sample; }
    MotorSample& sample() throw() { return m_sample; }

    //! Async calibration flag.
    /** Overwrite this method, returning true to enable async calibration (see knPicservo)
     * Note that the calibrateHook() method has to set m_sample.status to idle after successful calibration,
     * if used in async mode.
     */
    virtual bool calibrateAsync() const throw();
  protected:

    //! The actual calibration code
    /** Note that this code is called without any locks taken */
    virtual int calibrateHook();

  private:
    virtual int svc();

  protected:
    //! Pointer to the motor parameters.
    /** Note, that Motor does not take ownership. */
    MotorParameters const * m_params;
    MotorSample m_sample;

    MotorBase::DoubleVector m_positions;
    MotorBase::DoubleVector m_speeds;
    int m_indexOffset;
    double m_frequency;
  };
}
#endif // kn_MotorHelper_h
