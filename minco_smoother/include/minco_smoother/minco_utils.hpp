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

// Use trapezoidal velocity profile to get the total time of the path
double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len; 
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  // if(endV>max_vel_){
  if(endV>maxV){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  if(length>=critical_len){
    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
  }
  else{
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));
    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
  }
}

}

#endif // NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_