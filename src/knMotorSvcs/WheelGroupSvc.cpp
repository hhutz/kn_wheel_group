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

#include "WheelGroupSvc.h"
#include "WheelGroupJointProvider.h"
#include "knMotorSvcs/MotorSvcsParameters.h"

#include "knMotor/MotorBase.h"
#include "knMotor/WheelGroup.h"
#include "knMotor/WheelGroupSample.h"
#include "knMotorImpl/WheelGroupRepository.h"
#include "knMotorImpl/WheelGroupImpl.h"

#include "miro/Log.h"
#include "miro/Configuration.h"
#include "miro/RobotParameters.h"
#include "miro/TimeHelper.h"

#include <ace/Get_Opt.h>

namespace kn
{
  using namespace std;
  
  static const char* svcName = "WheelGroupSvc";

  WheelGroupSvc::WheelGroupSvc() :
    m_params(NULL),
    m_verbose(false),
    m_jointProvider(NULL)
  {
    MIRO_LOG_CTOR("kn::WheelGroupSvc");
  }

  WheelGroupSvc::~WheelGroupSvc()
  {
    MIRO_LOG_DTOR("kn::WheelGroupSvc");
  }

  int
  WheelGroupSvc::init(int argc, ACE_TCHAR *argv[])
  {
    MIRO_LOG(LL_NOTICE, "kn::WheelGroupSvc::init()");

    if (parseArgs(argc, argv) == -1)
      return -1;

    // get wheel group
    WheelGroup * wheelGroup = WheelGroupRepository::instance()->get(m_params->wheelGroupName);
    WheelGroupImpl * wheelGroupImpl  = dynamic_cast<WheelGroupImpl *>(wheelGroup);
    if (wheelGroupImpl == NULL) {
      MIRO_LOG(LL_CRITICAL, "kn::WheelGroupSvc::init() WheelGroup implementation not derived of type WheelGroupImpl.");
      return -1;
    }
   
    // get extended motor status service if requested

    // create corba interface
    m_jointProvider = new WheelGroupJointProvider(m_params->jointProvider, svcName, wheelGroupImpl);

    MIRO_LOG(LL_NOTICE, "kn::WheelGroupSvc::init() done");
    return 0;
  }

  int 
  WheelGroupSvc::info(ACE_TCHAR **bufferp, size_t len) const
  {
    static const char i[] = "kn::WheelGroupSvc Service \n";

    if (*bufferp == 0)
      *bufferp = ACE::strnew(i);
    else 
      ACE_OS_String::strncpy(*bufferp, i, len);

    return ACE_OS_String::strlen(*bufferp);
  }
   
  int 
  WheelGroupSvc::fini()
  {
    MIRO_LOG(LL_NOTICE, "kn::WheelGroupSvc::fini()");

    delete m_jointProvider;
    m_jointProvider = NULL;

    m_wheelGroup = NULL;

    MIRO_LOG(LL_NOTICE, "kn::WheelGroupSvc::fini() done");

    return 0;
  }

  int
  WheelGroupSvc::parseArgs(int& argc, char * argv[])
  {
    int rc = 0;
    int c;
    
    // initialize parameters with global instance
    m_params = WheelGroupSvcParameters::instance();
    // reset defaults
    WheelGroupSvcParameters defaultParams;
    *m_params = defaultParams;
    
    // initialize parameters from config file
    Miro::ConfigDocument * config = Miro::Configuration::document();
    config->setSection("Locomotion");
    config->getParameters("kn::WheelGroupSvcParameters", *m_params);

    // initialize parameters from command line
    ACE_Get_Opt get_opts (argc, argv, "n:w:v?");
  
    while ((c = get_opts()) != -1) {
      switch (c) {
      case 'n':
        m_params->svcName = get_opts.optarg;
        break;
      case 'w':
        m_params->wheelGroupName = get_opts.optarg;
        break;
      case 'v':
        ++m_verbose;
        break;
      case '?':
      default:
        ACE_OS::last_error(EINVAL);
        rc = -1;
      }
    }

    if (rc != 0) {
      cerr << "usage: " << argv[0] << " [-n name] [-vi?]\n"
           << "  -n <name> name of the interface instance\n"
           << "  -w <name> name of the wheel group instance\n"
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
ACE_FACTORY_NAMESPACE_DEFINE (knMotorSvcs, kn_WheelGroupSvc, kn::WheelGroupSvc);
ACE_STATIC_SVC_DEFINE (kn_WheelGroupSvc,
		       ACE_TEXT("WheelGroupSvc"),
		       ACE_SVC_OBJ_T,
		       &ACE_SVC_NAME(kn_WheelGroupSvc),
		       ACE_Service_Type::DELETE_THIS | ACE_Service_Type::DELETE_OBJ,
		       0);
ACE_STATIC_SVC_REQUIRE(kn_WheelGroupSvc);
