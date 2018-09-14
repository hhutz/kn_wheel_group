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

#ifndef kn_WheelGroupJointProvider_h
#define kn_WheelGroupJointProvider_h

#include "knMotorSvcs_Export.h"

#include "rapidIo/JointProvider.h"

#include "knDds/DdsTypedSupplier.h"

#include <boost/signals2.hpp>
#include <boost/shared_ptr.hpp>

namespace rapid
{
  namespace ext
  { 
    namespace arc
    {
      class WheelGroupSample;
    }
  }
}

namespace kn
{
  class WheelGroupJointProviderParameters;
  class WheelGroupSample;
  class WheelGroupImpl;

  class knMotorSvcs_Export WheelGroupJointProvider : public rapid::JointProvider
  {
    typedef kn::DdsTypedSupplier<rapid::ext::arc::WheelGroupSample> WheelGroupSampleSupplier;
    typedef boost::scoped_ptr<WheelGroupSampleSupplier> WheelGroupSupplierPtr;

  public:
    WheelGroupJointProvider(WheelGroupJointProviderParameters const& params,
                            std::string const& entityName,
                            WheelGroupImpl * wheelGroup);
    virtual ~WheelGroupJointProvider() throw();

    // direct callback from Wheelgroup
    void publishData(WheelGroupSample const& sample);
    void publishWheelGroupSample(WheelGroupSample const& sample);

  protected:
    WheelGroupImpl * m_wheelGroup;
    WheelGroupSupplierPtr m_wheelGroupSupplier;

    boost::signals2::scoped_connection m_sampleUpdateConnection;
    boost::signals2::scoped_connection m_wheelGroupSampleConnection;
  };
}
#endif // kn_WheelGroupJointProvider_h
