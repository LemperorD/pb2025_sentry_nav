// Copyright 2025 LemperorD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_
#define NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_

#include <cmath>

namespace minco_smoother
{

void normalizeAngle(double from, double & to){
  while (to - from > M_PI) to -= 2.0 * M_PI;
  while (to - from < -M_PI) to += 2.0 * M_PI;
}

}

#endif // NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_