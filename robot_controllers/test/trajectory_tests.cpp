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

// Author: Michael Ferguson

#include <gtest/gtest.h>
#include <robot_controllers/trajectory.h>
#include <robot_controllers/trajectory_spline_sampler.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

TEST(TrajectoryTests, test_unwind)
{
  robot_controllers::Trajectory t;
  t.points.resize(3);
  t.points[0].q.resize(3);
  t.points[0].q[0] = 3.0;
  t.points[0].q[1] = -3.0;
  t.points[0].q[2] = -3.0;
  t.points[1].q.resize(3);
  t.points[1].q[0] = -3.0;
  t.points[1].q[1] = 3.0;
  t.points[1].q[2] = 3.0;
  t.points[2].q.resize(3);
  t.points[2].q[0] = 3.0;
  t.points[2].q[1] = -3.0;
  t.points[2].q[2] = -3.0;

  std::vector<bool> continuous(3, false);
  continuous[0] = true;
  continuous[1] = true;

  // Test windup
  EXPECT_TRUE(windupTrajectory(continuous, t));
  // First joint should be wound up
  EXPECT_EQ(3.0, t.points[0].q[0]);
  EXPECT_EQ(3.2831853071795862, t.points[1].q[0]);
  EXPECT_EQ(3.0, t.points[2].q[0]);
  // Second joint should be wound up
  EXPECT_EQ(-3.0, t.points[0].q[1]);
  EXPECT_EQ(-3.2831853071795862, t.points[1].q[1]);
  EXPECT_EQ(-3.0, t.points[2].q[1]);
  // Third joint should be unmodified
  EXPECT_EQ(-3.0, t.points[0].q[2]);
  EXPECT_EQ(3.0, t.points[1].q[2]);
  EXPECT_EQ(-3.0, t.points[2].q[2]);

  // Test unwind
  EXPECT_TRUE(unwindTrajectoryPoint(continuous, t.points[0]));
  EXPECT_EQ(3.0,  t.points[0].q[0]);
  EXPECT_EQ(-3.0, t.points[0].q[1]);
  EXPECT_EQ(-3.0,  t.points[0].q[2]);
  EXPECT_TRUE(unwindTrajectoryPoint(continuous, t.points[1]));
  EXPECT_EQ(-3.0, t.points[1].q[0]);
  EXPECT_EQ(3.0,  t.points[1].q[1]);
  EXPECT_EQ(3.0, t.points[1].q[2]);
  EXPECT_TRUE(unwindTrajectoryPoint(continuous, t.points[2]));
  EXPECT_EQ(3.0,  t.points[2].q[0]);
  EXPECT_EQ(-3.0,  t.points[2].q[1]);
  EXPECT_EQ(-3.0, t.points[2].q[2]);


  // Make sure we catch mis-sized vectors
  t.points[1].q.resize(2);
  EXPECT_FALSE(windupTrajectory(continuous, t));

  continuous.resize(4);
  t.points[1].q.resize(3);
  EXPECT_FALSE(windupTrajectory(continuous, t));
}

TEST(TrajectoryTests, test_multiple_unwinds)
{
  robot_controllers::Trajectory t;
  t.points.resize(10);
  t.points[0].q.push_back(-3.0);
  t.points[1].q.push_back(3.0);
  t.points[2].q.push_back(1.0);
  t.points[3].q.push_back(-1.0);
  t.points[4].q.push_back(-3.0);
  t.points[5].q.push_back(3.0);
  t.points[6].q.push_back(1.0);
  t.points[7].q.push_back(-1.0);
  t.points[8].q.push_back(-3.0);
  t.points[9].q.push_back(3.0);

  std::vector<bool> continuous(1, true);

  // Test unwind
  EXPECT_TRUE(windupTrajectory(continuous, t));
  // First joint should be unwound
  EXPECT_EQ(-3.0, t.points[0].q[0]);
  EXPECT_EQ(-3.2831853071795862, t.points[1].q[0]);
  EXPECT_EQ(-5.2831853071795862, t.points[2].q[0]);
  EXPECT_EQ(-7.2831853071795862, t.points[3].q[0]);
  EXPECT_EQ(-9.2831853071795862, t.points[4].q[0]);
  EXPECT_EQ(-9.566370614359172, t.points[5].q[0]);
  EXPECT_EQ(-11.566370614359172, t.points[6].q[0]);
  EXPECT_EQ(-13.566370614359172, t.points[7].q[0]);
  EXPECT_EQ(-15.566370614359172, t.points[8].q[0]);
  EXPECT_EQ(-15.849555921538759, t.points[9].q[0]);
}

// Test trajectory with positions only
TEST(TrajectoryTests, test_pos)
{
  robot_controllers::Trajectory t;
  t.points.resize(2);
  t.points[0].q.resize(1);
  t.points[0].q[0] = 0.0;
  t.points[0].time = 0.0;
  t.points[1].q.resize(1);
  t.points[1].q[0] = 1.0;
  t.points[1].time = 1.0;

  boost::shared_ptr<robot_controllers::TrajectorySampler> sampler_;
  sampler_.reset(new robot_controllers::SplineTrajectorySampler(t));
  robot_controllers::TrajectoryPoint p0 = sampler_->sample(0.0);
  robot_controllers::TrajectoryPoint p1 = sampler_->sample(0.5);
  robot_controllers::TrajectoryPoint p2 = sampler_->sample(0.75);
  robot_controllers::TrajectoryPoint p3 = sampler_->sample(1.0);

  EXPECT_EQ(0.0, p0.q.size());
  EXPECT_EQ(0.5, p1.q[0]);
  EXPECT_EQ(0.75, p2.q[0]);
  EXPECT_EQ(1.0, p3.q[0]);
}

// Test trajectory with positions and velocities
TEST(TrajectoryTests, test_pos_vel)
{
  robot_controllers::Trajectory t;
  t.points.resize(2);
  t.points[0].q.resize(1);
  t.points[0].qd.resize(1);
  t.points[0].q[0] = 0.0;
  t.points[0].qd[0] = 1.0;
  t.points[0].time = 0.0;
  t.points[1].q.resize(1);
  t.points[1].qd.resize(1);
  t.points[1].q[0] = 1.0;
  t.points[1].qd[0] = 1.0;
  t.points[1].time = 1.0;

  boost::shared_ptr<robot_controllers::TrajectorySampler> sampler_;
  sampler_.reset(new robot_controllers::SplineTrajectorySampler(t));
  robot_controllers::TrajectoryPoint p0 = sampler_->sample(0.0);
  robot_controllers::TrajectoryPoint p1 = sampler_->sample(0.5);
  robot_controllers::TrajectoryPoint p2 = sampler_->sample(0.75);
  robot_controllers::TrajectoryPoint p3 = sampler_->sample(1.0);

  EXPECT_EQ(0.0, p0.q.size());
  EXPECT_EQ(0.5, p1.q[0]);
  EXPECT_EQ(0.75, p2.q[0]);
  EXPECT_EQ(1.0, p3.q[0]);
}

// Test trajectory with positions, velocities, and accelerations
TEST(TrajectoryTests, test_pos_vel_accel)
{
  robot_controllers::Trajectory t;
  t.points.resize(2);
  t.points[0].q.resize(1);
  t.points[0].qd.resize(1);
  t.points[0].qdd.resize(1);
  t.points[0].q[0] = 0.0;
  t.points[0].qd[0] = 1.0;
  t.points[0].qdd[0] = 0.0;
  t.points[0].time = 0.0;
  t.points[1].q.resize(1);
  t.points[1].qd.resize(1);
  t.points[1].qdd.resize(1);
  t.points[1].q[0] = 1.0;
  t.points[1].qd[0] = 1.0;
  t.points[1].qdd[0] = 0.0;
  t.points[1].time = 1.0;

  boost::shared_ptr<robot_controllers::TrajectorySampler> sampler_;
  sampler_.reset(new robot_controllers::SplineTrajectorySampler(t));
  robot_controllers::TrajectoryPoint p0 = sampler_->sample(0.0);
  robot_controllers::TrajectoryPoint p1 = sampler_->sample(0.5);
  robot_controllers::TrajectoryPoint p2 = sampler_->sample(0.75);
  robot_controllers::TrajectoryPoint p3 = sampler_->sample(1.0);

  EXPECT_EQ(0.0, p0.q.size());
  EXPECT_EQ(0.5, p1.q[0]);
  EXPECT_EQ(0.75, p2.q[0]);
  EXPECT_EQ(1.0, p3.q[0]);
}

// Test trajectory with velocities only
TEST(TrajectoryTests, test_vel)
{
  robot_controllers::Trajectory t;
  t.points.resize(2);
  t.points[0].qd.resize(1);
  t.points[0].qd[0] = 0.0;
  t.points[0].time = 0.0;
  t.points[1].qd.resize(1);
  t.points[1].qd[0] = 1.0;
  t.points[1].time = 1.0;

  boost::shared_ptr<robot_controllers::TrajectorySampler> sampler_;
  sampler_.reset(new robot_controllers::SplineTrajectorySampler(t));
  robot_controllers::TrajectoryPoint p0 = sampler_->sample(0.0);
  robot_controllers::TrajectoryPoint p1 = sampler_->sample(0.5);
  robot_controllers::TrajectoryPoint p2 = sampler_->sample(0.75);
  robot_controllers::TrajectoryPoint p3 = sampler_->sample(1.0);

  EXPECT_EQ(1, p1.qd.size());

  EXPECT_EQ(0.0, p0.qd.size());
  EXPECT_EQ(0.5, p1.qd[0]);
  EXPECT_EQ(0.75, p2.qd[0]);
  EXPECT_EQ(1.0, p3.qd[0]);
}

// Test trajectory with velocities and accelerations
TEST(TrajectoryTests, test_vel_accel)
{
  robot_controllers::Trajectory t;
  t.points.resize(2);
  t.points[0].qd.resize(1);
  t.points[0].qdd.resize(1);
  t.points[0].qd[0] = 0.0;
  t.points[0].qdd[0] = 1.0;
  t.points[0].time = 0.0;
  t.points[1].qd.resize(1);
  t.points[1].qdd.resize(1);
  t.points[1].qd[0] = 1.0;
  t.points[1].qdd[0] = 1.0;
  t.points[1].time = 1.0;

  boost::shared_ptr<robot_controllers::TrajectorySampler> sampler_;
  sampler_.reset(new robot_controllers::SplineTrajectorySampler(t));
  robot_controllers::TrajectoryPoint p0 = sampler_->sample(0.0);
  robot_controllers::TrajectoryPoint p1 = sampler_->sample(0.5);
  robot_controllers::TrajectoryPoint p2 = sampler_->sample(0.75);
  robot_controllers::TrajectoryPoint p3 = sampler_->sample(1.0);

  // Test other numbers??
  EXPECT_EQ(1, p1.qd.size());
  EXPECT_EQ(1, p1.qdd.size()); 

  EXPECT_EQ(0.0, p0.qd.size());
  EXPECT_EQ(0.5, p1.qd[0]);
  EXPECT_EQ(0.75, p2.qd[0]);
  EXPECT_EQ(1.0, p3.qd[0]);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
