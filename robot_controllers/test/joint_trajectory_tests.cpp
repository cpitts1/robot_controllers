/*
 * Copyright (c) 2014-2015, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Cappy Pitts

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <robot_controllers/follow_joint_trajectory.h>
#include <robot_controllers_interface/controller_manager.h>
#include <iostream>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

TEST(JointTrajectoryTests, init_controller_test)
{
  ros::Time::init();
  ros::NodeHandle nh("head_controller/follow_joint_trajectory");
  ros::NodeHandle nh2("robot_driver");
  std::vector<std::string> names;
  ASSERT_TRUE(nh.getParam("joints", names));
  EXPECT_EQ(2, names.size());
  EXPECT_EQ("head_pan_joint", names[0]);

  std::vector<std::string> controller_params;
  nh2.getParam("default_controllers", controller_params);
  EXPECT_EQ(7, controller_params.size());

  /*
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectoryPoint p0, p1;
  trajectory_msgs::JointTrajectory t_msg;

  p0.positions.push_back(0.0);
  p0.positions.push_back(1.0);
  p0.time_from_start = ros::Duration(1.0);
  p1.positions.push_back(2.0);
  p1.positions.push_back(3.0);
  p1.time_from_start = ros::Duration(2.0);
  t_msg.points.push_back(p0);
  t_msg.points.push_back(p1);
  t_msg.joint_names.push_back("head_pan_joint");
  t_msg.joint_names.push_back("head_tilt_joint");

  std::vector<std::string> joints;
  joints.push_back("head_pan_joint");
  joints.push_back("head_tilt_joint");

  goal.trajectory = t_msg;
  goal.goal_time_tolerance = ros::Duration(0.0);
  */

  robot_controllers::ControllerManager *manager = new robot_controllers::ControllerManager();
  boost::shared_ptr<robot_controllers::FollowJointTrajectoryController> controller;
  controller.reset(new robot_controllers::FollowJointTrajectoryController());

  EXPECT_TRUE(manager);

  //EXPECT_EQ(0, manager->init(nh2));
  EXPECT_EQ(0, controller->init(nh, manager));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_trajectory_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
