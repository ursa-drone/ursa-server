/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: TKruse
 *********************************************************************/
#include <dwa_local_planner/my_trajectory_cost_function.h>

namespace dwa_local_planner {

    bool MyTrajectoryCostFunction::prepare(std::vector<geometry_msgs::PoseStamped> global_plan){
        global_plan_ = global_plan;
    }

    double MyTrajectoryCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj){
        //Setup
        double x_end, y_end, th_end;
        traj.getEndpoint(x_end, y_end, th_end);
        double x_diff, y_diff, distance_sq;

        //Iterate over the global plan - find a close point - use that to score
        int i=global_plan_.size();
        std::vector<geometry_msgs::PoseStamped>::iterator poseIt;
        for (poseIt=global_plan_.begin() ; poseIt < global_plan_.end(); poseIt++, i--){
        geometry_msgs::PoseStamped& w = *poseIt;
        x_diff = x_end-w.pose.position.x;
        y_diff = y_end-w.pose.position.y;
        distance_sq = x_diff*x_diff + y_diff*y_diff;
        if (distance_sq<=0.01){
          return i;
        }
        }
        return -1.0;
    }

}

