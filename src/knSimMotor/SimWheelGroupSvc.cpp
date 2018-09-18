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

#include "SimWheelGroupSvc.h"
#include "SimMotor.h"
#include "SimWheelGroup.h"
#include "SimMotorParameters.h"

#include "knMotorImpl/WheelGroupRepository.h"

#include "miro/Log.h"
#include "miro/Configuration.h"
#include "miro/RobotParameters.h"
#include "miro/TimeHelper.h"
#include "miro/ReactorTask.h"

#include <ace/Get_Opt.h>

namespace kn
{
  using namespace std;

  SimWheelGroupSvc::SimWheelGroupSvc() :
    m_params(NULL),
    m_verbose(false),
    m_wheelGroup(NULL)
  {
    MIRO_LOG_CTOR("kn::SimWheelGroupSvc");
    m_reactorTask.activate();
  }

  SimWheelGroupSvc::~SimWheelGroupSvc()
  {
    MIRO_LOG_DTOR("kn::SimWheelGroupSvc");
  }

  int
  SimWheelGroupSvc::init(int argc, ACE_TCHAR *argv[])
  {
    MIRO_LOG(LL_NOTICE, "kn::SimWheelGroupSvc::init()");

    if (parseArgs(argc, argv) == -1)
      return -1;

    // create wheel group
    SimWheelGroupParameters const& params = m_params->wheelGroup;
    {
      vector<MotorParameters>::const_iterator first, last = params.motors.end();

      for (first = params.motors.begin(); first != last; ++first) {
        //m_motors.push_back(new SimMotor(first.base()));
        m_motors.push_back(new SimMotor(&(*first)));
      }
    }

    WheelGroup::WheelParametersVector wParams;
    {
      vector<WheelParameters>::const_iterator first, last = params.wheels.end();

      for (first = params.wheels.begin(); first != last; ++first) {
        //wParams.push_back(first.base());
        wParams.push_back(&(*first));
      }
    }
    m_wheelGroup =
      new SimWheelGroup(m_motors, &params, wParams);

    reactor(m_reactorTask.reactor()); // Event demultiplexer running it's own thread
    m_timerId = reactor()->schedule_timer(this, NULL,
                                          m_params->wheelGroup.statusInterval,
                                          m_params->wheelGroup.statusInterval);

    WheelGroupRepository::instance()->add(m_params->svcName, m_wheelGroup);


    MIRO_LOG(LL_NOTICE, "kn::SimWheelGroupSvc::init() done");
    return 0;
  }

  int
  SimWheelGroupSvc::info(ACE_TCHAR **bufferp, size_t len) const
  {
    static const char i[] = "kn::SimWheelGroupSvc Service \n";

    if (*bufferp == 0)
      *bufferp = ACE::strnew(i);
    else
      ACE_OS_String::strncpy(*bufferp, i, len);

    return ACE_OS_String::strlen(*bufferp);
  }

  int
  SimWheelGroupSvc::fini()
  {
    MIRO_LOG(LL_NOTICE, "kn::SimWheelGroupSvc::fini()");

    WheelGroupRepository::instance()->remove(m_params->svcName);

    if (m_timerId != -1) {
      reactor()->cancel_timer(m_timerId);
      m_timerId = -1;
    }

    // clean up ReactorTask
    m_reactorTask.shutdown(true);

    m_wheelGroup = NULL;

    SimMotorVector::const_iterator first, last = m_motors.end();

    for (first = m_motors.begin(); first != last; ++first) {
      delete *first;
    }

    m_motors.clear();

    MIRO_LOG(LL_NOTICE, "kn::SimWheelGroupSvc::fini() done");

    return 0;
  }

  int
  SimWheelGroupSvc::handle_timeout(ACE_Time_Value const& now, void const * params)
  {
    double frequency =
      1. / ((double)m_params->wheelGroup.statusInterval.sec() +
            m_params->wheelGroup.statusInterval.usec() / 1000000.);

    m_wheelGroup->setTime(now);
    m_wheelGroup->executeTimeSlice(frequency);

    return 0;
  }

  int
  SimWheelGroupSvc::parseArgs(int& argc, char * argv[])
  {
    int rc = 0;
    int c;

    // initialize parameters with global instance
    m_params = SimWheelGroupSvcParameters::instance();
    // reset defaults
    SimWheelGroupSvcParameters defaultParams;
    *m_params = defaultParams;

    // initialize parameters from config file
    Miro::ConfigDocument * config = Miro::Configuration::document();
    config->setSection("Locomotion");
    config->getParameters("kn::SimWheelGroupSvcParameters", *m_params);

    // initialize parameters from command line
    ACE_Get_Opt get_opts(argc, argv, "n:v?");

    while ((c = get_opts()) != -1) {
      switch (c) {
      case 'v':
        ++m_verbose;
        break;

      case 'n':
        m_params->svcName = get_opts.opt_arg();
        break;

      case '?':
      default:
        ACE_OS::last_error(EINVAL);
        rc = -1;
      }
    }

    if (rc != 0) {
      cerr << "usage: " << argv[0] << " [-n name] [-vi?]\n"
           << "  -n <name> name of the service instance\n"
           << "  -v verbose mode\n"
           << "  -? help: print this text and stop" << endl;
    }

    if (m_verbose) {
      cerr << "Parameters: " << endl
           << *m_params << endl;
    }

    return rc;
  }

}

// Create an object that will insert the <Service> into the list
// of statically linked services that the <ACE_Service_Config> will
// process at run-time.
ACE_FACTORY_NAMESPACE_DEFINE(knSimMotor, kn_SimWheelGroupSvc, kn::SimWheelGroupSvc);
ACE_STATIC_SVC_DEFINE(kn_SimWheelGroupSvc,
                      ACE_TEXT("SimWheelGroupSvc"),
                      ACE_SVC_OBJ_T,
                      &ACE_SVC_NAME(kn_SimWheelGroupSvc),
                      ACE_Service_Type::DELETE_THIS | ACE_Service_Type::DELETE_OBJ,
                      0);
ACE_STATIC_SVC_REQUIRE(kn_SimWheelGroupSvc);
