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


// For min/max
#include <algorithm>

#include <boost/bind.hpp>

#include <math.h>
#include <stdio.h>

#include <geometry_msgs/Twist.h>

#include "nav_pcontroller/nav_pcontroller.h"


double BasePController::p_control(double x, double p, double limit)
{
  return (x > 0) ? std::min(p*x, limit) : std::max(p*x, -limit);
}


double BasePController::limit_acc(double x, double x_old, double limit)
{
  x = std::min(x, x_old + limit);
  x = std::max(x, x_old - limit);
  return x;
}

BasePController::BasePController()
  : n_("~"),
    move_base_actionserver_(n_, "move_base", false)
{
  parseParams();

  move_base_actionserver_.registerGoalCallback(boost::bind(&BasePController::newMoveBaseGoal, this));
  move_base_actionserver_.registerPreemptCallback(boost::bind(&BasePController::preemptMoveBaseGoal, this));
  
  sub_goal_ = n_.subscribe("goal", 1, &BasePController::newGoal, this);
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  vx_=0; vy_=0; vth_=0;  // \todo: get from base driver

  goal_set_ = false;

  move_base_actionserver_.start();
}

BasePController::~BasePController()
{
}

void BasePController::main()
{
  ros::Rate loop(loop_rate_);
  ros::AsyncSpinner spinner(1);

  spinner.start();
  while(n_.ok()) {
    if(goal_set_)
      cycle();
    loop.sleep();
  }
}

void BasePController::parseParams()
{
  n_.param<double>("xy_tolerance", xy_tolerance_, 0.005);
  n_.param<double>("th_tolerance", th_tolerance_, 0.005);

  double tmp_fail_timeout;
  n_.param<double>("fail_timeout", tmp_fail_timeout, 5.0);
  fail_timeout_ = ros::Duration(tmp_fail_timeout);

  n_.param<double>("fail_velocity", fail_velocity_, 0.02);
  n_.param<double>("vel_ang_max", vel_ang_max_, 0.2);
  n_.param<double>("vel_lin_max", vel_lin_max_, 0.2);
  n_.param<double>("acc_ang_max", acc_ang_max_, 0.4);
  n_.param<double>("acc_lin_max", acc_lin_max_, 0.4);
  n_.param<int>("loop_rate", loop_rate_, 30);
  n_.param<double>("p", p_, 1.2);
  n_.param<bool>("keep_distance", keep_distance_, true);

  n_.param<std::string>("global_frame", global_frame_, "map");
  n_.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  //CHANGED
  //  dist_control_.setFootprint(0.42, -0.3075, 0.3075, -0.42, 0.0);
  /*(0.309, -0.43, 0.43, -0.309, 0.0);*/
  double front, rear, left, right, tolerance;
  std::string stName;
  n_.param("speed_filter_name", stName, std::string("/speed_filter"));
  n_.param(stName + "/footprint/left", left, 0.309);
  n_.param(stName + "/footprint/right", right, -0.309);
  n_.param(stName + "/footprint/front", front, 0.43);
  n_.param(stName + "/footprint/rear", rear, -0.43);
  n_.param(stName + "/footprint/tolerance", tolerance, 0.0);

  
  n_.param<double>("laser_watchdog_timeout", laser_watchdog_timeout_, 0.2);

  dist_control_.setFootprint(front, rear, left, right, tolerance);
}

void BasePController::newMoveBaseGoal()
{
  move_base_msgs::MoveBaseGoal::ConstPtr msg = move_base_actionserver_.acceptNewGoal();

  // To be able to reconfigure the base controller on the fly, whe read the parameters whenever we receive a goal
  parseParams(); 

  newGoal(msg->target_pose);
  ROS_INFO("received goal: %f %f %f", x_goal_, y_goal_, th_goal_);
}

void BasePController::preemptMoveBaseGoal()
{
  boost::mutex::scoped_lock curr_lock(lock);

  goal_set_ = false;
  stopRobot();
  move_base_actionserver_.setPreempted();
}

void BasePController::newGoal(const geometry_msgs::PoseStamped &msg)
{
  boost::mutex::scoped_lock curr_lock(lock);

  geometry_msgs::PoseStamped goal;

  try
  {
    tf_.waitForTransform(
        global_frame_, msg.header.frame_id, msg.header.stamp, ros::Duration(1.0));
    tf_.transformPose(global_frame_, msg, goal);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet %s",ex.what());
    return;
  }

  x_goal_ = goal.pose.position.x;
  y_goal_ = goal.pose.position.y;

  // th = atan2(r21/2,r11/2)
  const geometry_msgs::Quaternion &q = goal.pose.orientation;
  th_goal_ = atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y -q.z*q.z);

  ROS_INFO("got goal: %f %f %f", x_goal_, y_goal_, th_goal_);
  
  low_speed_time_ = ros::Time::now();

  goal_set_ = true;
}

void BasePController::newGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(move_base_actionserver_.isActive())
    preemptMoveBaseGoal();

  newGoal(*msg);
}

//! retrieves tf pose and updates (x_now_, y_now_, th_now_)
bool BasePController::retrieve_pose()
{
  tf::StampedTransform global_pose;
  try
  {
    tf_.lookupTransform(global_frame_, base_link_frame_, ros::Time(0), global_pose);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("no localization information yet %s",ex.what());
    return false;
  }

  // find out where we are now
  x_now_ = global_pose.getOrigin().x();
  y_now_ = global_pose.getOrigin().y();

  // th = atan2(r_21/2, r_11/2)
  const tf::Quaternion &q = global_pose.getRotation();
  th_now_ = atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());

  return true;
}

void BasePController::compute_p_control()
{
  //compute differences (world space)
  double x_diff = x_goal_ - x_now_;
  double y_diff = y_goal_ - y_now_;

  // \todo: clean this ugly code
  double th_diff = fmod(th_goal_ - th_now_, 2*M_PI);
  if(th_diff > M_PI) th_diff = th_diff - 2*M_PI;
  if(th_diff < -M_PI) th_diff = th_diff + 2*M_PI;

  // transform to robot space
  double dx =  x_diff*cos(th_now_) + y_diff*sin(th_now_);
  double dy = -x_diff*sin(th_now_) + y_diff*cos(th_now_);
  double dth = th_diff;

  // do p-controller with limit (robot space)
  double vel_x =  p_control(dx, p_, vel_lin_max_);
  double vel_y =  p_control(dy, p_, vel_lin_max_);
  double vel_th = p_control(dth, p_, vel_ang_max_);

  // limit acceleration (robot space)
  vel_x  = limit_acc(vel_x, vx_, acc_lin_max_/loop_rate_);
  vel_y  = limit_acc(vel_y, vy_, acc_lin_max_/loop_rate_);
  vel_th = limit_acc(vel_th, vth_, acc_ang_max_/loop_rate_);

  // store resulting velocity
  vx_  = vel_x;
  vy_  = vel_y;
  vth_ = vel_th;
}

#ifndef max
#define max(A,B) (A) > (B) ? (A) : (B)
#endif
void BasePController::cycle()
{
  boost::mutex::scoped_lock curr_lock(lock);

  if(!retrieve_pose()) {
    stopRobot();
    return;
  }

  if ( !dist_control_.fresh_scans(laser_watchdog_timeout_) ) {
    stopRobot();
    return;
  }

  compute_p_control();
  
  if (keep_distance_)
    dist_control_.compute_distance_keeping(&vx_, &vy_, &vth_);

  sendVelCmd(vx_, vy_, vth_);

  if(comparePoses(x_goal_, y_goal_, th_goal_, x_now_, y_now_, th_now_)) {
    
    if(move_base_actionserver_.isActive())
      move_base_actionserver_.setSucceeded();
    goal_set_ = false;
    stopRobot();
  }
  // Sort of a bad hack. It might be a bad idea to 'unify' angular
  // and linear velocity and just take te maximum.
  double velocity = max(sqrt(vx_ * vx_+ vy_ * vy_) , vth_);

  if( velocity < fail_velocity_ )
  {
    if( ros::Time::now() - low_speed_time_ > fail_timeout_ )
    {
      goal_set_ = false;
      stopRobot();
      
      if(move_base_actionserver_.isActive())
        move_base_actionserver_.setAborted();
      return;
    }
  }
  else
    low_speed_time_ = ros::Time::now();

  if(move_base_actionserver_.isActive())
  {
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position.header.stamp = ros::Time::now();
    feedback.base_position.header.frame_id = global_frame_;
    feedback.base_position.pose.position.x = x_now_;
    feedback.base_position.pose.position.y = y_now_;
    feedback.base_position.pose.position.z = 0.0;
    feedback.base_position.pose.orientation = tf::createQuaternionMsgFromYaw(th_now_);
    move_base_actionserver_.publishFeedback(feedback);
  }
}

#define ANG_NORM(a) atan2(sin((a)),cos((a)))

void BasePController::stopRobot()
{
  sendVelCmd(0.0,0.0,0.0);
}

void BasePController::sendVelCmd(double vx, double vy, double vth)
{
  geometry_msgs::Twist cmdvel;

  cmdvel.linear.x = vx;
  cmdvel.linear.y = vy;
  cmdvel.angular.z = vth;

  pub_vel_.publish(cmdvel);
}


bool BasePController::comparePoses(double x1, double y1, double a1,
                                   double x2, double y2, double a2)
{
  bool res;
  if((fabs(x2-x1) <= xy_tolerance_) &&
     (fabs(y2-y1) <= xy_tolerance_) &&
     (fabs(ANG_NORM(ANG_NORM(a2)-ANG_NORM(a1))) <= th_tolerance_))
    res = true;
  else
    res = false;
  return(res);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nav_pcontroller");

  BasePController pc;
  pc.main();
  return 0;
}
