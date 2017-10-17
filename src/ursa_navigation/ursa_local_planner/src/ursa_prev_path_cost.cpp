
/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <ursa_local_planner/ursa_prev_path_cost.h>
//DEBUG
#include <ros/ros.h>
#include <iostream>
using namespace std;

namespace ursa_local_planner {


bool UrsaPrevPathCostFunction::init(base_local_planner::Trajectory previous_result_traj){
        previous_result_traj_ = previous_result_traj;
    }

inline bool logically_equal(double a, double b){
        //cout << "a: " << a << ", b: " << b << endl;
        //cout << "std::abs(a-b)<DBL_EPSILON: " << (std::abs(a-b)<DBL_EPSILON) << endl;
        return std::abs(a-b)<DBL_EPSILON;
    }

double UrsaPrevPathCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
    return 0;
        // cout << "UrsaPrevPathCostFunction" << endl;

        double px, py, pth, x, y ,th;
        traj.getEndpoint(x, y, th);
        
        if (previous_result_traj_.getPointsSize()){
            previous_result_traj_.getEndpoint(px, py, pth);
            if ( logically_equal(px, x) && logically_equal(py, y) ){
                //cout << "le -- " << x << ", " << y << ", " << th << endl;
                return 0;
            }
        }

        // cout << "UrsaPrevPathCostFunction -- 10" << endl;
        return 0;
    }

}