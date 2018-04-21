/*
 * Copyright (c) 2009, Ingo Kresse <kresse@in.tum.de>, U. Klank
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@b This is a P controller for navigation. Yeah, this line is a todo...

@section topics ROS topics

Subscribes to (name/type):
- @b "/tf" tf/tfMessage: robot's pose in the "map" frame
- @b "~goal" geometry_msgs/PoseStamped : goal for the robot.

Publishes to (name / type):
- @b "/cmd_vel" geometry_msgs/Twist : velocity commands to robot
- @b "state" nav_robot_actions/MoveBaseState : current planner state (e.g., goal reached, no path) ??? does it

Actionlib (name / type):
- @b "~move_base/goal" move_base_msgs/MoveBaseActionGoal : similar to the move_base interface

@section parameters ROS parameters
  - @b "global_frame" (string) : global map frame, default: "map"
  - @b "base_link_frame" (string) : base frame, default: "base_link"
  - @b "xy_tolerance" (double) : Goal distance tolerance (how close the robot must be to the goal before stopping), default: 0.05 m
  - @b "th_tolerance" (double) : Goal rotation tolerance (how close the robot must be to the goal before stopping), default: 0.05 rad
  - @b "vel_lin_max" (double) : maximum linear velocity, default: 0.2 m/s
  - @b "vel_ang_max" (double) : maximum angular velocity, default: 0.2 rad/s
  - @b "acc_lin_max" (double) : maximum linear acceleration, default: 0.1 m/s^2
  - @b "acc_ang_max" (double) : maximum angular acceleration, default: 0.1 rad/s^2
  - @b "loop_rate" (int) : rate at which the control loop runs, default: 30 s^-1
  - @b "p" (double) : P controller value, default: 1.0
  - @b "keep_distance" (bool) : use BaseDistance to avoid obstacles or not, default: true

*/

#ifndef NAV_PCONTROLLER_H
#define NAV_PCONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include "tf/transform_listener.h"
#include <actionlib/server/simple_action_server.h>

#include <string>

#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "BaseDistance.h"

class BasePController {
private:

  double xy_tolerance_, th_tolerance_;
  ros::Duration fail_timeout_;
  double fail_velocity_;
  double vel_ang_max_, vel_lin_max_, acc_ang_max_, acc_lin_max_, p_;
  int loop_rate_;

  double vx_, vy_, vth_;
  double x_goal_, y_goal_, th_goal_;
  double x_now_, y_now_, th_now_;
  bool goal_set_, keep_distance_;

  double laser_watchdog_timeout_;

  std::string global_frame_;
  std::string base_link_frame_;

  BaseDistance dist_control_;

  ros::NodeHandle n_;
  tf::TransformListener tf_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_vel_;

  ros::Time low_speed_time_;

  boost::mutex lock;

  void newGoal(const geometry_msgs::PoseStamped &msg);
  void newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void cycle();
  void stopRobot();
  void sendVelCmd(double vx, double vy, double vth);
  bool comparePoses(double x1, double y1, double a1,
                    double x2, double y2, double a2);

  bool retrieve_pose();
  double p_control(double x, double p, double limit);
  double limit_acc(double x, double x_old, double limit);
  void compute_p_control();

  void parseParams();

  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> move_base_actionserver_;

  void newMoveBaseGoal();
  void preemptMoveBaseGoal();

public:
  BasePController();
  ~BasePController();
  void main();
};

#endif
