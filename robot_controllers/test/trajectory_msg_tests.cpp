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

#include <gtest/gtest.h>
#include <robot_controllers/trajectory.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>

// Test trajectory msg with positions only
TEST(TrajectoryMsgTests, test_pos_msg)
{
  ros::Time::init();
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

  robot_controllers::Trajectory t_new;

  EXPECT_TRUE(trajectoryFromMsg(t_msg, joints, &t_new));
  EXPECT_EQ(2, t_new.size());
  EXPECT_EQ(2, t_new.points[0].q.size());
  EXPECT_EQ(0.0, t_new.points[0].q[0]);
  EXPECT_EQ(2.0, t_new.points[1].q[0]);
  EXPECT_EQ(0, t_new.points[0].qd.size());
  EXPECT_EQ(0, t_new.points[0].qdd.size());
}


// Test trajectory msg with positions and velocities
TEST(TrajectoryMsgTests, test_pos_vel_msg)
{
  ros::Time::init();
  trajectory_msgs::JointTrajectoryPoint p0, p1;
  trajectory_msgs::JointTrajectory t_msg;

  p0.positions.push_back(0.0);
  p0.positions.push_back(1.0);
  p0.velocities.push_back(1.0);
  p0.velocities.push_back(1.0);
  p0.time_from_start = ros::Duration(1.0);
  p1.positions.push_back(2.0);
  p1.positions.push_back(3.0);
  p1.velocities.push_back(2.0);
  p1.velocities.push_back(2.0);
  p1.time_from_start = ros::Duration(2.0);
  t_msg.points.push_back(p0);
  t_msg.points.push_back(p1);
  t_msg.joint_names.push_back("head_pan_joint");
  t_msg.joint_names.push_back("head_tilt_joint");

  std::vector<std::string> joints;
  joints.push_back("head_pan_joint");
  joints.push_back("head_tilt_joint");

  robot_controllers::Trajectory t_new;

  EXPECT_TRUE(trajectoryFromMsg(t_msg, joints, &t_new));
  EXPECT_EQ(2, t_new.size());
  EXPECT_EQ(2, t_new.points[0].q.size());
  EXPECT_EQ(2, t_new.points[0].qd.size());
  EXPECT_EQ(2.0, t_new.points[1].q[0]);
  EXPECT_EQ(1.0, t_new.points[0].qd[0]);
  EXPECT_EQ(0, t_new.points[0].qdd.size());
}


// Test trajectory msg with positions, velocities, and accelerations
TEST(TrajectoryMsgTests, test_pos_vel_accel_msg)
{
  ros::Time::init();
  trajectory_msgs::JointTrajectoryPoint p0, p1;
  trajectory_msgs::JointTrajectory t_msg;

  p0.positions.push_back(0.0);
  p0.positions.push_back(1.0);
  p0.velocities.push_back(1.0);
  p0.velocities.push_back(1.0);
  p0.accelerations.push_back(0.0);
  p0.accelerations.push_back(0.0);
  p0.time_from_start = ros::Duration(1.0);
  p1.positions.push_back(2.0);
  p1.positions.push_back(3.0);
  p1.velocities.push_back(2.0);
  p1.velocities.push_back(2.0);
  p1.accelerations.push_back(1.0);
  p1.accelerations.push_back(1.0);
  p1.time_from_start = ros::Duration(2.0);
  t_msg.points.push_back(p0);
  t_msg.points.push_back(p1);
  t_msg.joint_names.push_back("head_pan_joint");
  t_msg.joint_names.push_back("head_tilt_joint");

  std::vector<std::string> joints;
  joints.push_back("head_pan_joint");
  joints.push_back("head_tilt_joint");

  robot_controllers::Trajectory t_new;

  EXPECT_TRUE(trajectoryFromMsg(t_msg, joints, &t_new));
  EXPECT_EQ(2, t_new.size());
  EXPECT_EQ(2, t_new.points[0].q.size());
  EXPECT_EQ(2, t_new.points[0].qd.size());
  EXPECT_EQ(2, t_new.points[0].qdd.size());
  EXPECT_EQ(2.0, t_new.points[1].q[0]);
  EXPECT_EQ(1.0, t_new.points[0].qd[0]);
  EXPECT_EQ(0.0, t_new.points[0].qdd[1]);
}

// Test trajectory msg with velocities only
TEST(TrajectoryMsgTests, test_vel_msg)
{
  ros::Time::init();
  trajectory_msgs::JointTrajectoryPoint p0, p1;
  trajectory_msgs::JointTrajectory t_msg;

  p0.velocities.push_back(0.0);
  p0.velocities.push_back(1.0);
  p0.time_from_start = ros::Duration(1.0);
  p1.velocities.push_back(2.0);
  p1.velocities.push_back(3.0);
  p1.time_from_start = ros::Duration(2.0);
  t_msg.points.push_back(p0);
  t_msg.points.push_back(p1);
  t_msg.joint_names.push_back("head_pan_joint");
  t_msg.joint_names.push_back("head_tilt_joint");

  std::vector<std::string> joints;
  joints.push_back("head_pan_joint");
  joints.push_back("head_tilt_joint");

  robot_controllers::Trajectory t_new;

  EXPECT_TRUE(trajectoryFromMsg(t_msg, joints, &t_new));
  EXPECT_EQ(2, t_new.size());
  EXPECT_EQ(2, t_new.points[0].qd.size());
  EXPECT_EQ(0.0, t_new.points[0].qd[0]);
  EXPECT_EQ(2.0, t_new.points[1].qd[0]);
  EXPECT_EQ(0, t_new.points[0].q.size());
  EXPECT_EQ(0, t_new.points[0].qdd.size());
}

// Test trajectory msg with velocities and accelerations
TEST(TrajectoryMsgTests, test_vel_accel_msg)
{
  ros::Time::init();
  trajectory_msgs::JointTrajectoryPoint p0, p1;
  trajectory_msgs::JointTrajectory t_msg;

  p0.velocities.push_back(0.0);
  p0.velocities.push_back(1.0);
  p0.accelerations.push_back(1.0);
  p0.accelerations.push_back(1.0);
  p0.time_from_start = ros::Duration(1.0);
  p1.velocities.push_back(2.0);
  p1.velocities.push_back(3.0);
  p1.accelerations.push_back(2.0);
  p1.accelerations.push_back(2.0);
  p1.time_from_start = ros::Duration(2.0);
  t_msg.points.push_back(p0);
  t_msg.points.push_back(p1);
  t_msg.joint_names.push_back("head_pan_joint");
  t_msg.joint_names.push_back("head_tilt_joint");

  std::vector<std::string> joints;
  joints.push_back("head_pan_joint");
  joints.push_back("head_tilt_joint");

  robot_controllers::Trajectory t_new;

  EXPECT_TRUE(trajectoryFromMsg(t_msg, joints, &t_new));
  EXPECT_EQ(2, t_new.size());
  EXPECT_EQ(2, t_new.points[0].qd.size());
  EXPECT_EQ(2, t_new.points[0].qdd.size());
  EXPECT_EQ(2.0, t_new.points[1].qd[0]);
  EXPECT_EQ(1.0, t_new.points[0].qdd[0]);
  EXPECT_EQ(0, t_new.points[0].q.size());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
