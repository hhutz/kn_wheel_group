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

#ifndef kn_WheelGroupSvc_h
#define kn_WheelGroupSvc_h

#include "knMotorSvcs_Export.h"

#include <ace/Service_Object.h>
#include <ace/Service_Config.h>


// forward declarations

namespace kn
{
  class WheelGroup;
  class WheelGroupJointProvider;
  class WheelGroupI;
  class WheelGroupSvcParameters;
  class WheelGroupSample;

  class knMotorSvcs_Export WheelGroupSvc : public ACE_Service_Object
  {
  public:
     WheelGroupSvc();
    virtual ~WheelGroupSvc();

    virtual int init(int argc, ACE_TCHAR *argv[]);
    virtual int info(ACE_TCHAR **src, size_t len) const;
    virtual int fini();

  private:
    int parseArgs(int& argc, char * argv[]);

    WheelGroupSvcParameters * m_params;
    int m_verbose;

    WheelGroup * m_wheelGroup;
    WheelGroupJointProvider * m_jointProvider;


    WheelGroupSvc(WheelGroupSvc const&);
    WheelGroupSvc& operator = (WheelGroupSvc const&);
  };
}

// Declare both static and dynamic services.
ACE_STATIC_SVC_DECLARE_EXPORT(knMotorSvcs, kn_WheelGroupSvc)
ACE_FACTORY_DECLARE(knMotorSvcs, kn_WheelGroupSvc)

#endif // kn_WheelGroupSvc_h
