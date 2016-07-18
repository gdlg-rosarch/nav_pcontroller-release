/*
 * Copyright (c) 2009, Ingo Kresse <kresse@in.tum.de>
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


#include "nav_pcontroller/speed_filter.h"

void SpeedFilter::input_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist t = *msg;

  dist_control_.compute_distance_keeping(&(t.linear.x), &(t.linear.y), &(t.angular.z));
  pub_vel_.publish(t);
}


SpeedFilter::SpeedFilter() : n_("~")
{
  sub_vel_ =  n_.subscribe("/input_vel", 1, &SpeedFilter::input_vel, this);
  pub_vel_ =  n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  //CHANGED
  //  dist_control_.setFootprint(0.42, -0.3075, 0.3075, -0.42, 0.0);
  double front, rear, left, right, tolerance;
  n_.param("footprint/left", left, 0.309);
  n_.param("footprint/right", right, -0.309);
  n_.param("footprint/front", front, 0.43);
  n_.param("footprint/rear", rear, -0.43);
  n_.param("footprint/tolerance", tolerance, 0.0);
  dist_control_.setFootprint(front, rear, left, right, tolerance);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "speed_filter");

  SpeedFilter sf;

  ros::spin();

  return 0;
}
