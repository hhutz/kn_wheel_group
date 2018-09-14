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

#include "HwReactor.h"
#include "miro/Log.h"

namespace kn
{
  Singleton<HwReactor> HwReactor::instance;

  HwReactor::HwReactor(ACE_Sched_Params *pschedp, bool shutdownOnException, int size) :
      ReactorTask(pschedp, shutdownOnException, size)
  {
    MIRO_LOG_CTOR("kn::HwReactor");
  }

  HwReactor::~HwReactor()
  {
    MIRO_LOG_DTOR("kn::HwReactor");
  }
}
#if defined (ACE_HAS_EXPLICIT_STATIC_TEMPLATE_MEMBER_INSTANTIATION)
template kn::Singleton<kn::HwReactor> * kn::Singleton<kn::HwReactor>::s_instance;
#endif /* ACE_HAS_EXPLICIT_STATIC_TEMPLATE_MEMBER_INSTANTIATION */
