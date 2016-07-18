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

/**

@b This is the BaseDistance utility class for avoiding obstacles!
It figures out if the robot should slow down or move freely based on laser scan data.
If there are points in the laser (lasers) that are closer than safe distance
the robot should start slowing down, completely brake, or even back up.

@section topic ROS topics

Subscribes to (name/type):
- @b "~laser_1" sensor_msgs/LaserScan: laser scan from first laser
- @b "~laser_2" sensor_msgs/LaserScan: laser scan from second laser if available

Publishes to (name / type):
- @b "/visualization_marker" visualization_msgs/Marker : markers for base footprint and laser nearest point
- @b "~laser_points" sensor_msgs/PointCloud : laser scanner points from combined lasers & interpolated blind spots
- @b "~debug_channels" std_msgs/Float64MultiArray : debug array for velocity gradient debugging of grad method

@section parameters ROS parameters
  - @b "odom_frame" (string) : odometry frame, default: "/odom"
  - @b "base_link_frame" (string) : base frame, default: "/base_link"
  - @b "n_lasers" (int) : number of lasers on robot's base, default: 2
  - @b "slowdown_far" (double) : distance from footprint edges to closest obstacle point from when to start slowing, default: 0.30 m
  - @b "slowdown_near" (double) : distance from footprint edges to closest obstacle point from when to slow aggressively, default: 0.15 m
  - @b "safety_dist" (double) : distance in meters when to brake when moving with current velocity, default: 0.10 m
  - @b "repelling_dist" (double) : distance from footprint edges to closest obstacle point from when to start backing up, default: 0.20 m
  - @b "repelling_gain" (double) : gain for repelling speed, default: 0.5
  - @b "repelling_gain_max" (double) : maximum repelling speed gain, default: 0.015
  - @b "complete_blind_spots" (bool) : interpolate the angles where lasers can't see or not, default: true
  - @b "blind_spot_threshold" (double) : distance from base_link_frame to the end of a blind spot, default: 0.85 m
  - @b "marker_size" (double) : size of markers in Rviz, default: 0.1 m

*/

#ifndef BASE_DISTANCE_H
#define BASE_DISTANCE_H

#include <string>
#include <vector>

#include <math.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>


class BaseDistance {

public:

  /**
   * @brief The Vector2 helper class to store 2D coordinates
   */
  class Vector2 {
  public:
    double x, y;
    Vector2() {}
    Vector2(double x_, double y_) : x(x_), y(y_) {}

    double len2() { return x * x + y * y; }
    double len() { return sqrt(len2()); }
  };

  /**
   * @brief Gets the parameters from ROS parameter server and initializes arguments with that
   */
  BaseDistance();

  /**
   * @brief Sets front_, rear_, left_, right_, tolerance_, and early_reject_distance_.
   */
  void setFootprint(double front, double rear, double left, double right, double tolerance);

  /**
   * @brief Sets safety_dist_, slowdown_near_, slowdown_far_, and d_ = 1 / rate.
   * \todo rename to use underscores
   */
  void setSafetyLimits(double safety_dist, double slowdown_near, double slowdown_far, double rate);

  /**
   * @brief Computes the translation and rotation from one frame to another
   * @param[in] from The name of the parent frame
   * @param[in] to The name of the child frame
   * @param[in] time Timestamp of the transform
   * @param[out] x X component of translation
   * @param[out] y Y component of translation
   * @param[out] th Angle of rotation around the Z axis
   * @return false if no transform could be looked up
   */
  bool compute_pose2d(const char* from, const char* to, const ros::Time time, double *x, double *y, double *th);

  /**
   * @brief Calculates the distance to closest of @p points from nearest footprint wall
   * The distance is calculated not from the current position of @a base_link_frame_,
   * but the adjusted with @p dx, @p dy and @p dth one, because in the next timestamp
   * the robot will have moved a little.
   * @param points Coordinates in @a odom_frame_ (laser data)
   * @param nearest If nearest is not false, updates it with the closest point
   * (in adjusted @a base_link_frame_)
   * @param dx The X component of adjustment vector of robot coordinates
   * (adjustment vector is defined in @a base_link_frame_)
   * @param dy The Y component of adjustment vector
   * @param dth Theta component of adjustment vector
   * @return Distance to closest of @p points from nearest footprint wall for next time frame
   */
  double distance(std::vector<Vector2> &points, Vector2 *nearest, double dx=0, double dy=0, double dth=0);

  /**
   * @brief Sets @a vx_last_, @a vy_last and @a vth_last_ to input parameters, adjusting them
   * @param[out] vx Velocity to move with, if there were no obstacles
   * @param[out] vy Y component
   * @param[out] vth Theta component
   */
  void compute_distance_keeping(double *vx, double *vy, double *vth);


private:

  std::string odom_frame_, base_link_frame_;

  /**
   * @brief d_ duration of one time frame, i.e. dt or Tdot
   */
  double d_;

  /**
   * @brief n_lasers_ Number of laser scanners that create the base scan.
   * Note: @a laser_points_ is of size 2, so currently only up to 2 lasers supported.
   */
  int n_lasers_;

  /**
   * @brief Footprint of the robot, coordinates of front edge, back edge, etc.
   * in the footprint coordinate system.
   * E.g. front = 0.33, rear = -0.33, left = 0.33, right = -0.33
   */
  double front_, rear_, left_, right_;

  /**
   * @brief tolerance_ Error in the footprint measurements in meters.
   */
  double tolerance_;

  /**
   * @brief slowdown_far_ Distance from footprint edges to closest obstacle point from when to start slowing
   */
  double slowdown_far_;

  /**
   * @brief slowdown_near_ Distance from footprint edges to closest obstacle point from when to slow aggressively
   */
  double slowdown_near_;

  /**
   * @brief safety_dist_ Distance in meters when to brake when moving with current velocity.
   */
  double safety_dist_;

  /**
   * @brief repelling_dist_ Distance from footprint edges to closest obstacle point from when to start backing up
   */
  double repelling_dist_, repelling_gain_, repelling_gain_max_;

  /**
   * @brief early_reject_distance_ From how many meters to discard laser measurements.
   * Laser is used to avoid obstacles, so if the obstacles are far away
   * they're not relevant for the time being.
   */
  double early_reject_distance_;

  /**
   * @brief complete_blind_spots_ If the laser/s have blind spots, wheter to complete them.
   * The completion will be through interpolation.
   */
  bool complete_blind_spots_;

  /**
   * @brief blind_spot_threshold_ Distance from base_link_frame to the end of a blind spot.
   * Non-parallel lines always meet somewhere, so the blind spot is a triangle.
   * If the obstacles are further away than the triangle, means they are
   * outside of the blind spot, so there is no need to triangulate.
   */
  double blind_spot_threshold_;

  /**
   * @brief blind_spots_ A vector of interpolated coordinates for the blind spots
   * Defined in the @a odom_frame_
   */
  std::vector<Vector2> blind_spots_;

  /**
   * @brief laser_points_ Data from lasers filtered and converted into the odom frame.
   * The size is 2 for 2 lasers, i.e. supports up to 2 lasers only.
   */
  boost::shared_ptr<std::vector<Vector2> > laser_points_[2];
  //boost::shared_ptr<std::vector<Vector2> > current_points_; //!< should replace the points argument of distance(), project(), grad() and brake().

  /**
   * @brief nearest_ The nearest point to the base
   */
  Vector2 nearest_;

  /**
   * @brief rob_x_, rob_y_, rob_th_ Pose of the @a base_link_frame_ in @a odom_frame_
   */
  double rob_x_, rob_y_, rob_th_;

  /**
   * @brief vx_last_, vy_last_, vth_last_ Robot velocity to drive with in next time frame
   */
  double vx_last_, vy_last_, vth_last_;

  /**
   * @brief mode_ Defined the movement mode for the base.
   * Can be one of the modes defined in the cpp file:
   * MODE_FREE - move freely,
   * MODE_PROJECTING - slow down slowly,
   * MODE_HARD_PROJECTING - slow down faster,
   * MODE_BRAKING - brake right now, command 0 velocity,
   * MODE_REPELLING - move away from obstacle.
   */
  int mode_;


  ros::NodeHandle n_;

  /**
   * @brief marker_size_ When publishing points to RViz, in meters
   */
  double marker_size_;

  ros::Publisher marker_pub_;
  ros::Publisher laser_points_pub_;
  ros::Publisher debug_pub_;

  tf::TransformListener tf_;

  ros::Subscriber laser_subscriptions_[2];

  boost::mutex lock;

  /**
   * @brief early_reject_distance_ = diameter + slowdown_far_ + tolerance_ + movement_tolerance;
   */
  void calculateEarlyRejectDistance();

  /**
   * @brief Interpolates @p n points between @p pt1 and @p pt2 and adds them to @a blind_spots_
   * Interpolation is linear and component-wise: separately on X and Y coordinates
   * @p pt1 and @p pt2 are defined in @a odom_frame_
   * @return true if @a blind_spots_ was updated
   */
  bool interpolateBlindPoints(int n, const Vector2 &pt1, const Vector2 &pt2);

  /**
   * @brief laserCallback Sets the @a laser_points_ to filtered data from lasers in the odom frame.
   * Completes blind spots if @a complete_blind_spots is set to true.
   * @param index For when there are multiple lasers on the base, to index over them
   * @param scan Input ROS message from the laser with index @p index
   */
  void laserCallback(int index, const sensor_msgs::LaserScan::ConstPtr& scan);

  /**
   * @brief Applies ridig transformation to the input vector, i.e. rotation and translation
   * I.e. @p gets transformed into a coordinate system defined by @p x, @p y and @p th
   * @param v Input vector
   * @param x x component of the translation vector
   * @param y y component of the translation vector
   * @param th Angle of rotation
   * @return Transformed vector v' = Rv + t
   */
  Vector2 transform(const Vector2 &v, double x, double y, double th);

  /**
   * @brief If distance to closest point when moving with given velocity is short, brake.
   * Given velocity here is given through @p vx, @p vy and @p vth.
   * Short means smaller than @a safety_dist_.
   * Adjusts @a mode_
   * @param points Laser data in @a odom_frame_
   * @param[in,out] vx Current velocity of the robot from which to start braking when obstacle is close
   * @param[in,out] vy Y component
   * @param[in,out] vth Theta component
   * @return Distance to closes obstacle point from robot pose in next time frame with velocity @p vx
   */
  double brake(std::vector<Vector2> &points, double *vx, double *vy, double *vth);

  /**
   * @brief Calculates the gradient of the distance to the closes point of @p points
   * Is it increasing? Is it decreasing? How fast?
   * @param points
   * @param gx X component of gradient
   * @param gy Y component of gradient
   * @param gth Distance gradient in robot's angular velocity
   * @return Distance to closest point from robot footprint walls in current robot pose
   */
  double grad(std::vector<Vector2> &points, double *gx, double *gy, double *gth);

  /**
   * @brief Adapts the velocity given in parameters to slow down or back up
   * Adjusts @a mode_, @p vx, @p vy and @p vth
   * @param points Points from the laser in @a odom_frame_
   * @param[in,out] vx Current velocity in X
   * @param[in,out] vy Current velocity in Y
   * @param[in,out] vth Current velocity in Theta
   * @return Distance to closest point from robot footprint walls in current robot pose
   */
  double project(std::vector<Vector2> &points, double *vx, double *vy, double *vth);

  /**
   * @brief Publishes a red cube at the coordinates of @p pt using @a marker_pub_
   * @param pt Point in odom_frame_
   */
  void publishLaserMarker(const Vector2 &pt, const std::string &ns, int id=0);

  /**
   * @brief Publishes @a nearest_ as a cube or a sphere of corresponding color
   * @a nearest_ is defined in base_link_frame_
   */
  void publishNearestPoint();

  /**
   * @brief Publishes a purple cube at the position of the robot in odom_frame_
   * The size of the cube is the size of the footprint given through @a front_ etc.
   */
  void publishBaseMarker();

  /**
   * @brief Publishes coordinates in @p points as a point cloud
   * Uses @a laser_points_pub_ to publish
   * @param points Coordinates in @a odom_frame_
   */
  void publishPoints(const std::vector<Vector2> &points);
};

#endif
