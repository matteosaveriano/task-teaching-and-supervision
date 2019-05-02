/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#ifndef DMP_H_
#define DMP_H_

#include "ros/ros.h"
//#include "LWR_seed_control/DMP/LearnDMPFromDemo.h"
//#include "LWR_seed_control/DMP/GetDMPPlan.h"
//#include "LWR_seed_control/DMP/SetActiveDMP.h"
#include "lwr_seed_control/DMPTraj.h"
#include "lwr_seed_control/DMPData.h"
#include "lwr_seed_control/DMP/radial_approx.h"
#include "lwr_seed_control/DMP/fourier_approx.h"
#include "lwr_seed_control/DMP/linear_approx.h"
#include <math.h>

namespace  DMP_RosWrapper{

double calcPhase(const double curr_time, const double tau);

void learnFromDemo(const lwr_seed_control::DMPTraj &demo,
				   const std::vector<double> &k_gains,
				   const std::vector<double> &d_gains,
				   const int &num_bases,
                   std::vector<lwr_seed_control::DMPData> &dmp_list);

void generatePlan(const std::vector<lwr_seed_control::DMPData> &dmp_list,
				  const std::vector<double> &x_0,
				  const std::vector<double> &x_dot_0,
				  const double &t_0,
				  const std::vector<double> &goal,
				  const std::vector<double> &goal_thresh,
				  const double &seg_length,
				  const double &tau,
				  const double &total_dt,
				  const int &integrate_iter,
                  lwr_seed_control::DMPTraj &plan,
				  uint8_t &at_goal);

}
#endif /* DMP_H_ */
