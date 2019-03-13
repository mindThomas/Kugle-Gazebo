/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

/* Inspired from https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/blob/indigo-devel/hector_gazebo_plugins/src/gazebo_ros_force_based_move.cpp
 *
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_BALLBOT_ACCELERATION_CONTROL_HH
#define GAZEBO_ROS_BALLBOT_ACCELERATION_CONTROL_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class GazeboRosBallbotAccelerationControl : public ModelPlugin {

    public: 
      GazeboRosBallbotAccelerationControl();
      ~GazeboRosBallbotAccelerationControl();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishWorldGroundTruth(void);
      void publishOdometry(double step_time);
      void publishJointStates(void);
      float unwrap(float previous_angle, float new_angle);

      tf::Transform getTransformForMotion(double linear_vel_x, double angular_vel, double timeSeconds) const;

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      /// \brief A pointer to the Link, where force is applied
      physics::LinkPtr link_;
      physics::LinkPtr robotLink_;

      /// \brief The Link this plugin is attached to, and will exert forces on.
      private: std::string link_name_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Publisher joints_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      tf::Transform odom_transform_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string world_frame_;
      std::string contact_point_link_;
      std::string heading_link_;
      double odometry_rate_;
      bool publish_odometry_tf_;
      double cmd_timeout_;
      double acceleration_coefficient_;

      tf::Quaternion attitudeReference_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void QuaternionRefCallback(const geometry_msgs::Quaternion::ConstPtr& cmd_msg);

      double prev_x_vel_;
      double prev_y_vel_;
      double prev_yaw_;
      double prev_yaw_for_odometry_;

      bool alive_;
      common::Time last_odom_publish_time_;
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d last_odom_pose_;
#else
      math::Pose last_odom_pose_;
#endif

      common::Time prev_update_time_;

      ros::Time last_cmd_update_time_;
  };

}

#endif /* end of include guard: GAZEBO_ROS_BALLBOT_ACCELERATION_CONTROL_HH */
