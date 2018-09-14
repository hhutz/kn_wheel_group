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
 * Module: knMotor
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_HwReactor_h
#define kn_HwReactor_h

#include "knMotor_Export.h"
#include "knShare/Singleton.h"
#include "miro/ReactorTask.h"

namespace kn
{
  class knMotor_Export HwReactor : public Miro::ReactorTask
  {
  public:
    HwReactor(ACE_Sched_Params * pschedp = NULL, bool shutdownOnException = true, int size = 20);
    virtual ~HwReactor();

    //! Singleton instance of worker threads.
    static Singleton<HwReactor> instance;
  };

  typedef Singleton<HwReactor> HwReactorSingleton;
}

KNMOTOR_SINGLETON_DECLARATION(kn::Singleton<kn::HwReactor>);

#endif // kn_HwReactor_h




