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

#ifndef kn_WheelGroupImpl_h
#define kn_WheelGroupImpl_h

#include "knMotorImpl_Export.h"
#include "MotorGroupImpl.h"

#include "knMotor/WheelGroup.h"
#include "knMotor/WheelGroupSample.h"

#include <boost/signals2.hpp>

namespace kn
{
  //! Abstract motor group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knMotorImpl_Export WheelGroupImpl : public virtual WheelGroup,
    public virtual MotorGroupImpl
  {
  public:
    typedef boost::signals2::signal1<void, WheelGroupSample const&> WheelGroupSampleSignal;
    typedef std::vector<std::pair<std::string, std::vector<double > > > AuxVector;

    //! Ctor
    /** The wheel-group parameters can be passed on construction,
     * or explicitly set later. Parameters are considered constant though.
     * The class does not take ownership of the parameters.
     */
    WheelGroupImpl(MotorVector const& motors = MotorVector(),
                   WheelGroupParameters const * params = NULL,
                   WheelParametersVector const& wheelParams = WheelParametersVector());
    virtual ~WheelGroupImpl() throw();

    // Motor Group interface

    virtual WheelGroupParameters const * params() const throw()
    {
      return m_params;
    }
    virtual void setParams(MotorGroupParameters const * params) throw();

    using WheelGroup::cmdProfiles;
    using WheelGroup::cmdTrajectories;

    virtual int cmdProfiles(std::vector<MotorProfile> const& profiles);
    virtual int cmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& postions,
                                TrajectoryVector const& speeds);

    // Wheel Group interface

    virtual unsigned int numWheels() const throw()
    {
      return m_params->wheels.size();
    }
    virtual WheelParameters const * wheelParams(unsigned int index) const throw()
    {
      return m_wheelParams[index];
    }
    virtual void setWheelParams(unsigned int index, WheelParameters const * params) throw();

    //    virtual WheelGroupSample wheelGroupState(bool waitNextSample = false, ACE_Time_Value * timeout = NULL);
    //    virtual int futureWheelGroupState(WheelGroupFuture& state, ACE_Time_Value const& targetTime);

    virtual void collectMotorStatus(ACE_Condition_Recursive_Thread_Mutex& condition,
                                    ACE_Time_Value const& timestamp = ACE_Time_Value::zero);

    int indexOffset() const throw();

    WheelGroupSampleSignal& wheelGroupSampleSignal() throw()
    {
      return m_wheelGroupSampleSignal;
    }

    AuxVector const & getAuxValues() const
    {
      return m_auxValues;
    }

  protected:
    WheelGroupSample wheelGroupStateSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
                                         bool waitNextSample = false, ACE_Time_Value const * timeout = NULL);

    int collectFutureWheelGroupState(WheelGroupFuture& state, ACE_Time_Value const& targetTime);

    void setStateStop(StopMode mode, ACE_Time_Value const& now);
    int setStateCmdProfiles(std::vector<MotorProfile> const& profiles,
                            double curvature, double crabAngle,
                            ACE_Time_Value const& now);
    int setStateCmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& mPostions,
                                TrajectoryVector const& mSpeeds,
                                DoubleVector const& curvatures,
                                DoubleVector const& curvatureRates,
                                DoubleVector const& crabAngles,
                                DoubleVector const& crabRates,
                                DoubleVector const& speeds);

    virtual void onCommandCompletion() throw();

    void consumeTrajectoryPoints(unsigned int num,
                                 ACE_Time_Value const& now, ACE_Time_Value const& targetTime)
    throw();

    WheelGroupParameters const * m_params;
    std::vector<WheelParameters const *> m_wheelParams;

    WheelGroupSample m_wheelGroupSample;

    DoubleVector m_curvatures;
    DoubleVector m_curvatureRates;
    DoubleVector m_crabAngles;
    DoubleVector m_crabRates;
    DoubleVector m_speeds;

    int m_indexOffset;
    double m_frequency;

    WheelGroupSampleSignal m_wheelGroupSampleSignal;

    AuxVector m_auxValues;
  };
}
#endif // kn_WheelGroupImpl_h
