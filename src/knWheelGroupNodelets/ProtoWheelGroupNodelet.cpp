/* -*- C++ -*- *****************************************************************
 * Copyright (c) 2018 United States Government as represented by the
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
 * Module: knWheelGroupNodelets
 * Author: Hans Utz
 *
 *****************************************************************************/

#include "ProtoWheelGroupNodelet.h"

#include <pluginlib/class_list_macros.h>

namespace kn
{
  template class AceSvcNodelet<protoMotor::MotorInterface>;
}
PLUGINLIB_EXPORT_CLASS(kn::ProtoWheelGroupNodelet, nodelet::Nodelet)
