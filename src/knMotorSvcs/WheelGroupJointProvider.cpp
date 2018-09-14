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
 * Module: knMotorSvcs
 * Author: Hans Utz
 *
 *****************************************************************************/

#include "WheelGroupJointProvider.h"
#include "MotorSvcsParameters.h"

#include "knMotor/MotorBase.h"
#include "knMotor/WheelGroup.h"
#include "knMotor/WheelGroupSample.h"
#include "knMotorImpl/WheelGroupRepository.h"
#include "knMotorImpl/WheelGroupImpl.h"

#include "rapidExtArcDds/WheelGroupSample.h"
#include "rapidExtArcDds/WheelGroupSampleSupport.h"
#include "rapidExtArcDds/ExtArcConstants.h"

#include "rapidDds/JointSampleSupport.h"
#include "rapidDds/JointConfigSupport.h"
#include "rapidDds/RapidConstants.h"
#include "rapidDds/JointSample.h"

#include "rapidIo/BaseTypesIo.h"
#include "rapidUtil/RapidHelper.h"

#include "knShare/Functional.h"

#include <ace/Time_Value.h>

#include <ndds/ndds_utility_cpp.h>
#include <ndds/ndds_namespace_cpp.h>

namespace
{
  using namespace kn;
  using namespace rapid;

  DDS::Long knMotor2RapidJointsStatus(int knStatus)
  {
    DDS::Long rapidStatus = 0;
    
    if (kn::MotorBase::isError(KNMOTOR_ERR_OVERCURRENT, knStatus))
      rapidStatus |= JOINT_OVER_CURRENT;
    if (kn::MotorBase::isError(KNMOTOR_ERR_POS_ERROR, knStatus))
      rapidStatus |= JOINT_POSITION_ERROR;
    if (kn::MotorBase::isError(KNMOTOR_ERR_HW_ESTOP, knStatus))
      rapidStatus |= JOINT_ESTOP;
    if (kn::MotorBase::isError(KNMOTOR_ERR_TRAJ_EXEC, knStatus))
      rapidStatus |= JOINT_FAILURE;
    if (kn::MotorBase::isError(KNMOTOR_ERR_POWER_FAULT, knStatus))
      rapidStatus |= JOINT_FAILURE | JOINT_DISABLED;
     if (kn::MotorBase::isError(KNMOTOR_ERR_SW_ESTOP, knStatus))
      rapidStatus |= JOINT_FAILURE | JOINT_FROZEN;
   
    return rapidStatus;
  }
}

namespace kn
{
  using namespace std;
  using namespace rapid;


  /**
   * ctor
   */
  WheelGroupJointProvider::WheelGroupJointProvider(WheelGroupJointProviderParameters const& params,
                                                   std::string const& entityName,
                                                   WheelGroupImpl * wheelGroup) :
    JointProvider(params, entityName),
    m_wheelGroup(wheelGroup),
    m_wheelGroupSupplier(new WheelGroupSampleSupplier(rapid::ext::arc::WHEELGROUP_SAMPLE_TOPIC +
                                                      params.topicSuffix,
                                                      params.parentNode,
                                                      params.wheelGroupSampleProfile,
                                                      params.library,
                                                      entityName))
  {
    JointSample& sample = m_dataSupplier.event();

    WheelGroupImpl::AuxVector const& auxVector = m_wheelGroup->getAuxValues();
    sample.auxFloat.length(auxVector.size());
    for (unsigned int i = 0; i < auxVector.size(); ++i) {
      sample.auxFloat[i].name = strdup(auxVector[i].first.c_str());
      sample.auxFloat[i].values.length(auxVector[i].second.size());
    }

    rapid::RapidHelper::initHeader(m_wheelGroupSupplier->event().hdr);

    m_sampleUpdateConnection = 
      m_wheelGroup->wheelGroupSampleSignal().connect(kn::bind(&WheelGroupJointProvider::publishData, this, _1));
    m_wheelGroupSampleConnection = 
      m_wheelGroup->wheelGroupSampleSignal().connect(kn::bind(&WheelGroupJointProvider::publishWheelGroupSample, this, _1));
  }

  /**
   * dtor
   */
  WheelGroupJointProvider::~WheelGroupJointProvider() throw()
  {
    // disconnect integrateData
    m_sampleUpdateConnection.disconnect();
    // wait a bit, so hopefully no thread executing is publis-data anymore
    ACE_OS::sleep(ACE_Time_Value(0, 100000));
  }

  void 
  WheelGroupJointProvider::publishData(WheelGroupSample const& data)
  {
    JointSample& sample = m_dataSupplier.event();

    // todo: populate all channels
    RapidHelper::updateHeader(sample.hdr, data.timestamp);
    sample.hdr.statusCode = knMotor2RapidJointsStatus(data.status);

    sample.anglePos.length(data.motors.size());
    sample.angleVel.length(data.motors.size());
    for (unsigned int i = 0; i < data.motors.size(); ++i) {
      sample.anglePos[i] = data.motors[i].position;
      sample.angleVel[i] = data.motors[i].speed;
    }

    sample.status.length(data.motorStatus.size());
    if (!data.motorStatus.empty()) {
      for (unsigned int i = 0; i < data.motorStatus.size(); ++i) {
        sample.status[i] = knMotor2RapidJointsStatus(data.motorStatus[i]);
      }
    }
    sample.current.length(data.currents.size());
    if (!data.currents.empty()) {
      for (unsigned int i = 0; i < data.currents.size(); ++i) {
        sample.current[i] = data.currents[i];
      }
    }
    sample.temperature.length(data.temperatures.size());
    if (!data.temperatures.empty()) {
      for (unsigned int i = 0; i < data.temperatures.size(); ++i) {
        sample.temperature[i] = data.temperatures[i];
      }
    }

    WheelGroupImpl::AuxVector const& auxVector = m_wheelGroup->getAuxValues();
    for (unsigned int i = 0; i < auxVector.size(); ++i) {
      for (unsigned int j = 0; j < auxVector[i].second.size(); ++j) {
        sample.auxFloat[i].values[j] = auxVector[i].second[j];
      }
    }

    m_dataSupplier.sendEvent();
  }

  namespace {
    rapid::ext::arc::CtrlMode ctrlModes[] = {
      rapid::ext::arc::CTRL_POSITION,
      rapid::ext::arc::CTRL_TRAPECOID,
      rapid::ext::arc::CTRL_TRAJECTORY,
      rapid::ext::arc::CTRL_HOLD,
      rapid::ext::arc::CTRL_OFF
    };
    rapid::ext::arc::PositionMode positionModes[] = {
      rapid::ext::arc::POS_ABS,
      rapid::ext::arc::POS_REL,
      rapid::ext::arc::POS_NA
    };
  }

  void 
  WheelGroupJointProvider::publishWheelGroupSample(WheelGroupSample const& data)
  {
    ext::arc::WheelGroupSample& sample = m_wheelGroupSupplier->event();

    // todo: populate all channels
    RapidHelper::updateHeader(sample.hdr, data.timestamp, data.status);

    sample.targetTime    = RapidHelper::aceTimeValue2RapidTime(data.targetTime);

    sample.curvature           = data.curvature;
    sample.curvatureRate       = data.curvatureRate;
    sample.speed               = data.speed;
    sample.crabAngle           = data.crabAngle;
    sample.crabRate            = data.crabRate;

    sample.targetCurvature     = data.targetCurvature;
    sample.targetCurvatureRate = data.targetCurvatureRate;
    sample.targetSpeed         = data.targetSpeed;
    sample.targetCrabAngle     = data.targetCrabAngle;
    sample.targetCrabRate      = data.targetCrabRate;

    //    sample.motors            <<= data.motors;
    sample.motors.ensure_length(data.motors.size(), 32);
    for (int i = 0; i < sample.motors.length(); ++i) {

      // sample.motors[i].cmd = data.motors[i].cmd;
      sample.motors[i].cmd.ctrlMode = ctrlModes[data.motors[i].cmd.ctrlMode];
      sample.motors[i].cmd.posMode = positionModes[data.motors[i].cmd.posMode];
      sample.motors[i].cmd.position = data.motors[i].cmd.position;
      sample.motors[i].cmd.speed = data.motors[i].cmd.speed;
      sample.motors[i].cmd.acc = data.motors[i].cmd.acc;


      sample.motors[i].position = data.motors[i].position;
      sample.motors[i].speed = data.motors[i].speed;      
    }

    // don't know why compiler can't resolve <<= directly
    rapid::operator<<=(sample.motorStatus, data.motorStatus);
    rapid::operator<<=(sample.currents, data.currents);
    rapid::operator<<=(sample.temperatures, data.temperatures);

    m_wheelGroupSupplier->sendEvent();
  }
}
