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

#ifndef kn_SimWheelGroupSvc_h
#define kn_SimWheelGroupSvc_h

#include "knSimMotor_Export.h"

#include <miro/ReactorTask.h>

#include <ace/Service_Config.h>
#include <ace/Service_Object.h>

#include <vector>

namespace kn
{
  class SimMotor;
  class SimWheelGroup;
  class SimWheelGroupSvcParameters;

  class knSimMotor_Export SimWheelGroupSvc : public ACE_Service_Object
  {
  public:
    typedef std::vector<SimMotor *> SimMotorVector;

    SimWheelGroupSvc();
    virtual ~SimWheelGroupSvc();

    virtual int init(int argc, ACE_TCHAR *argv[]);
    virtual int info(ACE_TCHAR **src, size_t len) const;
    virtual int fini();

    virtual int handle_timeout(ACE_Time_Value const& now, void const * params);

  private:
    int parseArgs(int& argc, char * argv[]);

    SimWheelGroupSvcParameters * m_params;
    int m_verbose;

    Miro::ReactorTask m_reactorTask;

    SimMotorVector m_motors;
    SimWheelGroup * m_wheelGroup;
 
    int m_timerId;

    SimWheelGroupSvc(SimWheelGroupSvc const&);
    SimWheelGroupSvc& operator = (SimWheelGroupSvc const&);
  };
}

// Declare both static and dynamic services.
ACE_STATIC_SVC_DECLARE_EXPORT(knSimMotor, kn_SimWheelGroupSvc)
ACE_FACTORY_DECLARE(knSimMotor, kn_SimWheelGroupSvc)

#endif // kn_SimWheelGroupSvc_h
