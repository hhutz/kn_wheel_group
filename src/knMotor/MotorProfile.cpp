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

#include "MotorProfile.h"

#include <iostream>
#include <cmath>
#include <cassert>

namespace
{
  char const * const ctrlModes[] = {
    "CTRL_POSITION",
    "CTRL_TRAPECOID",
    "CTRL_TRAJECTORY",
    "CTRL_HOLD",
    "CTRL_OFF"
  };

  char const * const posModes[] = {
    "POS_ABS",
    "POS_REL",
    "POS_NA"
  };
}

namespace kn
{
  using namespace std;

  std::ostream& operator<< (std::ostream& ostr, MotorProfile const& rhs)
  {
    assert(rhs.ctrlMode >= 0 && rhs.ctrlMode < 5);
    assert(rhs.posMode >= 0 && rhs.posMode < 3);

    ostr << "{" << ctrlModes[rhs.ctrlMode]
         << ", " << posModes[rhs.posMode]
         << ", " << rhs.position * 180. / M_PI << "deg"
         << ", " << rhs.speed * 180. / M_PI << "deg/s"
         << ", " << rhs.acc * 180. / M_PI << "deg/(s*s)}";

    return ostr;
  }

  std::ostream& operator<< (std::ostream& ostr, MotorProfileVector const& rhs)
  {
    MotorProfileVector::const_iterator first, last = rhs.end();

    for (first = rhs.begin(); first != last; ++first) {
      if (first != rhs.begin())
        ostr << endl;

      ostr << "{ " << *first << "}";
    }

    return ostr;
  }
}
