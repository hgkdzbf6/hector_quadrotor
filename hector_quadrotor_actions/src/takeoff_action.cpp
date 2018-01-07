//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_uav_msgs/ControlMode.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_actions/base_action.h>


namespace hector_quadrotor_actions
{

class TakeoffActionServer
{
public:
  TakeoffActionServer(ros::NodeHandle nh)
      : takeoff_server_(nh, "action/takeoff", boost::bind(&TakeoffActionServer::takeoffActionCb, this, _1)),
        pose_client_(nh, "action/pose"),
		control_mode_pub_(nh.advertise<hector_uav_msgs::ControlMode>("control_mode", 10))
  {
    nh.param<double>("action_frequency", frequency_, 10.0);
    nh.param<double>("takeoff_height", takeoff_height_, 0.3);
    nh.param<double>("connection_timeout", connection_timeout_, 10.0);
    nh.param<double>("action_timout", action_timeout_, 30.0);
    if(!pose_client_.waitForServer(ros::Duration(connection_timeout_))){
      ROS_ERROR_STREAM("Could not connect to " << nh.resolveName("action/pose"));
    }
    takeoff_server_.get()->start();
  }

  //这个是和pose通信然后做的起飞控制,效果不好.
  void takeoffActionCb(const hector_uav_msgs::TakeoffGoalConstPtr &goal)
  {

    if(takeoff_server_.enableMotors(true))
    {

      hector_uav_msgs::PoseGoal pose_goal;
      pose_goal.target_pose = *takeoff_server_.getPose();
      pose_goal.target_pose.header.frame_id="world";
      pose_goal.target_pose.pose.position.z = takeoff_height_;
      pose_client_.sendGoal(pose_goal);
      pose_client_.waitForResult(ros::Duration(action_timeout_));

      if (pose_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
    	  //当起飞成功之后,发一个control mode包
    	  hector_uav_msgs::ControlMode msg;
    	  msg.header.frame_id="control_mode";
    	  msg.header.stamp=ros::Time(0);
    	  msg.mode=hector_uav_msgs::ControlMode::NORMAL_CONTROL;
        ROS_WARN("Takeoff succeeded");
        takeoff_server_.get()->setSucceeded();
        control_mode_pub_.publish(msg);
        return;
      }
    }
    ROS_WARN("Takeoff failed");
    takeoff_server_.get()->setAborted();
  }

private:
  actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_client_;
  hector_quadrotor_actions::BaseActionServer<hector_uav_msgs::TakeoffAction> takeoff_server_;
  ros::Publisher pose_pub_;
  ros::Publisher control_mode_pub_;

  double frequency_, takeoff_height_, connection_timeout_, action_timeout_;
};

} // namespace hector_quadrotor_actions


int main(int argc, char **argv)
{
  ros::init(argc, argv, "takeoff_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::TakeoffActionServer server(nh);

  ros::spin();

  return 0;
}
