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
 * Module: knMotorImpl
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_MotorGroupRepository_h
#define kn_MotorGroupRepository_h

#include "knMotorImpl_Export.h"
#include "knMotor/MotorGroup.h"

#include "miro/Repository.h"

namespace kn
{
  typedef Miro::Repository<MotorGroup> MotorGroupRepository;
}

KNMOTORIMPL_SINGLETON_DECLARE(ACE_Singleton, Miro::Repository<kn::MotorGroup>, ACE_SYNCH_RECURSIVE_MUTEX);

#endif // kn_MotorGroupRepositoryx_h
