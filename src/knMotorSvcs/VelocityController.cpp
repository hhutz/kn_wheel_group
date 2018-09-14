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

#include "VelocityController.h"
#include "knMotorSvcs/MotorSvcsParameters.h"

#include "rapidExtArcDds/ExtArcConstants.h"
#include "rapidExtArcDds/Float32Sample.h"
#include "rapidExtArcDds/Float32SampleSupport.h"
#include "rapidExtArcDds/Float32ConfigSupport.h"

#include "rapidIo/KeyTypeValue.h"
#include "rapidIo/BaseTypesIo.h"
#include "rapidIo/PositionIo.h"
#include "rapidExtArcIo/Float32Io.h"

#include "knMotorShare/WheelGroupFuture.h"

#include "knDds/DdsEventLoop.h"
#include "miro/Log.h"

#include <cmath>

namespace kn
{
  using namespace std;

  using namespace rapid;
  using namespace rapid::ext::arc;

  using kn::DdsSampleCallbackFunctionPtr;

  VelocityController::VelocityController(VelocityControllerParameters const& params, std::string const& entityName,
                                         double steerLimit,
                                         WheelGroup * wheelGroup) :
    m_params(params),
    m_entityName(entityName),
    m_configured(!m_params.useConfigTopic),
    m_wheelGroup(wheelGroup)
  {
    MIRO_LOG_CTOR("VelocityController");
    
    Float32Config::TypeSupport::initialize_data(&m_velocityControlConfig);

    // initialize gps config from parameters with some meaningful values
    m_velocityControlConfig <<= m_params.config;

    Vector2Vector wheelLocations;
    wheelLocations.resize(4); //wheelGroup->parameters().wheels.size());
//     for (unsigned int i=0; i<m_locomotorModel.numberOfWheels(); i++) {
//       WheelData wheel = m_locomotorModel.getWheel(i);
//       Vector2 pos(wheel.position[0], wheel.position[1]);
//       wheelLocations.push_back(pos);
//     }

    // TODO: why are fsd parameters different from regular locomotor parametrs?
    m_fsd.reset(new FastestSteerAndDriveCWL(m_params.fsdSampleFrequency,
                                        m_params.wheelRadius,
                                        wheelLocations));
    m_fsd->setTransition(m_params.fsdWheelAccel,
                         m_params.fsdSteerAccel,
                         m_params.fsdSteerRate);


    // start processing loop
    this->activate();

    MIRO_LOG_CTOR_END("VelocityController");
  }

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  VelocityController::~VelocityController()
  {
    this->thr_mgr()->cancel_task(this);
    this->wait();
    MIRO_LOG(LL_NOTICE, "VelocityController thread finished.");

    rapid::ext::arc::Float32Config::TypeSupport::finalize_data(&m_velocityControlConfig);
  }

  /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////

  void VelocityController::velocityControlConfig(rapid::ext::arc::Float32Config const * config)
  {
    rapid::ext::arc::Float32Config::TypeSupport::copy_data(&m_velocityControlConfig, config);
    m_configured = true;
  }

  void
  VelocityController::velocityControlUpdate(rapid::ext::arc::Float32Sample const * sample)
  {
    // input checking
    if (!m_configured || sample->data.length() < 6) {
      return;
    }

    // calculate speed, curvature & crabAngle from velocity vectors
    float x = sample->data[0];
    float y = sample->data[1];
    float yaw = sample->data[5];

    float pathTime = 0.5;
    float targetWheelSpeed = sqrt(x*x + y*y);
    if (x < 0.) {
      targetWheelSpeed *= -1;
      x *= -1.;
      y *= -1.;
    }
    float targetCrab = atan2(y, x);
    float targetCurvature = yaw / (targetWheelSpeed + 0.01);

    // calculate wheel motions
    if (m_wheelGroup->cmdStatus() == KNMOTOR_BUSY) {
        MIRO_LOG_OSTR(LL_ERROR, "LocomotorImpl::driveContinuous called while motor group is busy.");
        return;
    }
    if (m_wheelGroup->cmdStatus() < 0) {
      MIRO_LOG_OSTR(LL_ERROR,
                    "LocomotorImpl::driveContinuous called but motor group is reporting errors (" <<
                    m_wheelGroup->cmdStatus() << "): " << MotorBase::getErrorName(m_wheelGroup->cmdStatus()));
      return;
    }

    kn::WheelGroupFuture groupFuture;
    int ref = m_wheelGroup->futureWheelGroupState(groupFuture);

    if (ref < -1) {
      assert(MotorBase::isError(KNMOTOR_ERR_TIMEOUT, ref));
      MIRO_LOG_OSTR(LL_ERROR, "LocomotorImpl::driveContinuous timeout acquiring future wheel group state (" <<
                    ref << "): " << MotorBase::getErrorName(ref));
      return;
    }

    if (isnan(groupFuture.curvatureRate) ||
        isnan(groupFuture.speed)) {
      // we are moving and have no idea how.
      // can't do anything before this stops.
      MIRO_LOG(LL_ERROR, "We are moving and have no idea why. Locomotor can't do anything before this stops.");
      return;
    }

    ACE_Time_Value start = ACE_OS::gettimeofday();
    m_fsd->clearPath();
    m_fsd->initPath(groupFuture);

    // wheels are unaligned
    // calcualate an alignement path
    if (isnan(groupFuture.curvature)) {
      m_fsd->appendAlignmentPath(pathTime);
    }

    m_fsd->appendPath(pathTime, targetWheelSpeed, targetCurvature, targetCrab);

    if (!m_fsd->getPpmSpeeds().empty()) {
      // if wheel are aligned, add a stop path
      double endCurvature = m_fsd->getPpmCurvatures().back();
      double endCrab = m_fsd->getPpmCrabs().back();
      if (!isnan(endCurvature)) {
#ifndef STOP_WITHOUT_OVERSHOOT
        m_fsd->appendPath(NAN, 0., endCurvature, endCrab);
#else
        m_fsd->appendPath(NAN, 0., NANd, endCrab);
#endif
      }
    }
    ACE_Time_Value stop = ACE_OS::gettimeofday();

    // command wheel group

    //----------------------
    // collect data vectors from fsd
    //
    WheelGroup::TrajectoryVector const& ppmMotorPositions = m_fsd->getPpmMotorPositions();
    WheelGroup::TrajectoryVector const& ppmMotorSpeeds =  m_fsd->getPpmMotorSpeeds();
    WheelGroup::DoubleVector const& ppmCurvatures = m_fsd->getPpmCurvatures();
    WheelGroup::DoubleVector const& ppmCurvatureRates = m_fsd->getPpmCurvatureRates();
    WheelGroup::DoubleVector const& ppmSpeeds = m_fsd->getPpmSpeeds();
    WheelGroup::DoubleVector const& ppmCrabs = m_fsd->getPpmCrabs();
    WheelGroup::DoubleVector const& ppmCrabRates = m_fsd->getPpmCrabRates();

    int retVal = m_wheelGroup->cmdTrajectories(ref,
                                         m_fsd->getSampleFrequency(),
                                         ppmMotorPositions,
                                         ppmMotorSpeeds,
                                         ppmCurvatures,
                                         ppmCurvatureRates,
                                         ppmCrabs,
                                         ppmCrabRates,
                                         ppmSpeeds);

    // debug communications

    // path analysis
    bool printFsdDebug = false;
    bool printFsdWarnings = false;
    if (printFsdDebug || printFsdWarnings) {
      stringstream ostr;
      m_fsd->printWarnings(ostr, printFsdDebug);
      if (printFsdDebug || !ostr.str().empty())
        ostr << "wheel group future: " << groupFuture << endl
             << "targetCurvature: " << targetCurvature << endl
             << "targetCrab: " << targetCrab << endl
             << "targetWheelSpeed: " << targetWheelSpeed << endl;

      if (printFsdDebug) {
        ostr << "wheel group reference: " << ref << endl;
        ostr << "path length: " << ppmCurvatures.size() << endl;
        ostr << "fsd calc time: " << (stop - start) << endl;
      }

      if (!ostr.str().empty())
        cerr << ostr.str() << endl;
    }

    if (retVal != 0) {
      MIRO_LOG_OSTR(LL_ERROR, "WheelGroup could not set path point buffers (" << retVal << "): " << MotorBase::getErrorName(retVal));
    }
  }

  /////////////////////////////////////////////////////////////////

  int VelocityController::svc()
  {
    kn::DdsEventLoop eventLoop(m_entityName);

    // create subscribers
    MIRO_LOG(LL_NOTICE, "VelocityController: Connecting all readers.");
    
      typedef DdsSampleCallbackFunctionPtr<Float32Config, This, &This::velocityControlConfig> ConfigCallback;
      typedef DdsSampleCallbackFunctionPtr<Float32Sample, This, &This::velocityControlUpdate> SampleCallback;
      if (m_params.useConfigTopic) {
        eventLoop.connect<Float32Config, ConfigCallback>(this,
                          rapid::ext::arc::FLOAT32_CONFIG_TOPIC + m_params.topicSuffix,
                          m_params.parentNode,
                          m_params.configProfile,
                          m_params.library);
      }
      eventLoop.connect<Float32Sample, SampleCallback>(this,
                        rapid::ext::arc::FLOAT32_SAMPLE_TOPIC + 
                        m_params.topicSuffix,
                        m_params.parentNode,
                        m_params.dataProfile,
                        m_params.library);
    
    MIRO_LOG(LL_NOTICE, "VelocityController: Entering dds event-loop.");
    ACE_Thread_Manager * mgr = this->thr_mgr();
    while (!mgr->testcancel(mgr->thr_self())) { // dds processing loop
      
      // 10Hz checking for cancel
      eventLoop.processEvents(kn::microseconds(100000));
    }

    MIRO_LOG(LL_NOTICE, "VelocityController::svc() Exiting dds event-loop.");
    return 0;
  }
}
