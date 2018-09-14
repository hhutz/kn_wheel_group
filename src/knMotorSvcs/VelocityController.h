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

#ifndef kn_VelocityController_h
#define kn_VelocityController_h

#include "knMotorSvcs/MotorSvcsParameters.h"

#include "knMotor/WheelGroup.h"
#include "knCoordinatedWheelLocomotion/FastestSteerAndDriveCWL.h"

#include "rapidExtArcDds/Float32Config.h"
#include "rapidExtArcDds/Float32Sample.h"

#include "knShare/SmartPtr.h"

#include <ace/Task.h>


// Forward declarations
namespace rapid
{
  namespace ext
  {
    namespace arc
    {
      class Float32Sample;
    }
  }
}

namespace kn
{

  class VelocityController : public ACE_Task_Base
  {
    typedef VelocityController This;

  public:
    VelocityController(VelocityControllerParameters const& params, std::string const& entityName,
                       double steerLimit,
                       WheelGroup * WheelGroup);

    /** Disconnect from the supplier. */
    virtual ~VelocityController();
    
  protected:
    virtual int svc();

    // config callbacks

    void velocityControlConfig(rapid::ext::arc::Float32Config const * config);

    // sample callbacks

    void velocityControlUpdate(rapid::ext::arc::Float32Sample const * sample);

    VelocityControllerParameters m_params;
    std::string m_entityName;
    rapid::ext::arc::Float32Config m_velocityControlConfig;
    bool m_configured;

    WheelGroup* m_wheelGroup;
    shared_ptr<FastestSteerAndDriveCWL> m_fsd;
    
  };
}

#endif // kn_VelocityController_h
