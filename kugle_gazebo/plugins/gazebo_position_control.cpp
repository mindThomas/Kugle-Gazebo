/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
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

#include <gazebo_position_control.h>
#include <cmath>       /* isnan, sqrt */

namespace gazebo 
{

  GazeboRosPositionControl::GazeboRosPositionControl() {}

  GazeboRosPositionControl::~GazeboRosPositionControl() {}

  // Load the controller
  void GazeboRosPositionControl::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PositionControl missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_quaternion";
    if (!sdf->HasElement("commandTopic"))
    {
      ROS_WARN("PositionControl (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    world_frame_ = "world";
    if (!sdf->HasElement("worldFrame"))
    {
        ROS_WARN("PositionControl (ns = %s) missing <worldFrame>, "
                 "defaults to \"%s\"",
                 robot_namespace_.c_str(), world_frame_.c_str());
    }
    else
    {
        world_frame_ = sdf->GetElement("worldFrame")->Get<std::string>();
    }

    contact_point_link_ = "contact_point";
    if (!sdf->HasElement("contactPointLink"))
    {
      ROS_WARN("PositionControl (ns = %s) missing <contactPointLink>, "
          "defaults to \"%s\"",
               contact_point_link_.c_str(), contact_point_link_.c_str());
    }
    else
    {
        contact_point_link_ = sdf->GetElement("contactPointLink")->Get<std::string>();
    }

    this->link_ = parent->GetLink(contact_point_link_);

    if (!this->link_)
    {
      ROS_FATAL_STREAM("Could not find link for contact point: " << contact_point_link_ << "\n");
      return;
    }

  cmd_timeout_ = 1.0;
  if (!sdf->HasElement("cmdTimeout"))
  {
      ROS_WARN("PositionControl (ns = %s) missing <cmdTimeout>, "
               "defaults to %f",
               robot_namespace_.c_str(), cmd_timeout_);
  }
  else
  {
      cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();
  }

 
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
#endif

    prev_x_vel_ = 0;
    prev_y_vel_ = 0;
    prev_yaw_ = 0;
    prev_yaw_for_odometry_ = 0;
    alive_ = true;

    attitudeReference_ = tf::Quaternion(0,0,0,1); // set as unit quaternion

    odom_transform_.setIdentity();

#if (GAZEBO_MAJOR_VERSION >= 8)
      prev_update_time_ = parent_->GetWorld()->SimTime();
#else
      prev_update_time_ = parent_->GetWorld()->GetSimTime();
#endif

    last_cmd_update_time_ = ros::Time::now();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PositionControl (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Quaternion>(command_topic_, 1,
          boost::bind(&GazeboRosPositionControl::QuaternionRefCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosPositionControl::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosPositionControl::UpdateChild, this));

  }

  float GazeboRosPositionControl::unwrap(float previous_angle, float new_angle)
  {
      float d = new_angle - previous_angle;
      d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
      return previous_angle + d;
  }

  // Update the controller
  void GazeboRosPositionControl::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    /* Set command to 0 if no reference is set for a while (timeout) */
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif

      ros::Time current_ros_time = ros::Time::now();
      ros::Duration diff = current_ros_time - last_cmd_update_time_;
      if (diff.toSec() > cmd_timeout_) {
          attitudeReference_ = tf::Quaternion(0,0,0,1); // set as unit quaternion
          //robotLink_->SetRelativePose(math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0.2, 0, 0)));

          /*ROS_INFO_STREAM("Links:");
          for (auto& link : link_->GetChildJointsLinks())
          {
              ROS_INFO_STREAM(link->GetName());
          }*/
          last_cmd_update_time_ = current_ros_time;
      }

      // Convert attitude reference to roll, pitch and yaw
      tfScalar yaw, pitch, roll;
      tf::Matrix3x3 mat(attitudeReference_);
      mat.getEulerYPR(yaw, pitch, roll);

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = parent_->WorldPose();

    ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

    double error = angular_vel.Z() - rot_;

ROS_INFO("Angular vel 1 error = %2.3f", error);

    link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));

    float yaw = pose.Rot().Yaw();

    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    link_->AddRelativeForce(ignition::math::Vector3d((x_ - linear_vel.X())* force_x_velocity_p_gain_,
                                                     (y_ - linear_vel.Y())* force_y_velocity_p_gain_,
                                                     0.0));
#else
    /*math::Pose pose = parent_->GetWorldPose();
    math::Vector3 linear_vel = parent_->GetWorldLinearVel();
    math::Vector3 angular_vel = parent_->GetWorldAngularVel();
    float yaw_ball = pose.rot.GetYaw();*/

    //ROS_INFO("angular   vel = %2.3f,\t accel = %2.3f", angular_vel.z, angular_accel.z);
    //ROS_INFO("rot_ = %2.3f", rot_);

    /*if (!std::isnan(error))
        link_->AddTorque(math::Vector3(0.0, 0.0, -error * torque_yaw_velocity_p_gain_));*/
    //link_->AddTorque(math::Vector3(0.0, 0.0, rot_));
    //link_->AddRelativeTorque(math::Vector3(0.0, 0.0, rot_));

    /*if (!std::isnan(linear_vel.x) && !std::isnan(linear_vel.y))
    link_->AddRelativeForce(math::Vector3((x_ - linear_vel.x)* force_x_velocity_p_gain_,
                                          (y_ - linear_vel.y)* force_y_velocity_p_gain_,
                                          0.0));*/

    //link_->AddForce(math::Vector3(x_, y_, 0.0));


    //link_->SetTorque(math::Vector3(0.0, 0.0, rot_));
    //link_->SetForce(math::Vector3(x_, y_, 0.0));
    /*if (parent_->GetLink("base_link"))
          parent_->GetLink("base_link")->SetForce(math::Vector3(x_, y_, 0.0));
    if (parent_->GetLink("robot"))
        parent_->GetLink("robot")->SetForce(math::Vector3(x_, y_, 0.0));*/

      /* Determine acceleration from current attitude reference
       * This comes from the linear tilt-only MPC model
       * - an approximate linear relationship matching the steady state dynamics
       */
      // First we need to transform the attitude reference into heading frame
      tf::Quaternion xAxis = attitudeReference_ * tf::Quaternion(1,0,0,0) * attitudeReference_.inverse();
      double RobotYaw = atan2(xAxis.y(), xAxis.x());
      tf::Quaternion q_heading(0,0,sin(RobotYaw/2),cos(RobotYaw/2));
      tf::Quaternion q_tilt = q_heading.inverse() * attitudeReference_; //  tilt in heading frame
                             //attitudeReference_ * q_heading.inverse(); // tilt in inertial frame
      if (q_tilt.w() < 0)
          q_tilt = q_tilt.inverse(); // invert q_tilt to take shortest rotation

      // Now we use the x and y component from this quaternion to estimate the acceleration in heading frame
      double accel_x_ref_heading_frame = acceleration_coefficient_ * q_tilt.y();
      double accel_y_ref_heading_frame = -acceleration_coefficient_ * q_tilt.x();

      // Rotate acceleration reference into inertial frame by rotatin with heading/yaw
      double accel_x_ref = cos(yaw)*accel_x_ref_heading_frame - sin(yaw)*accel_y_ref_heading_frame;
      double accel_y_ref = sin(yaw)*accel_x_ref_heading_frame + cos(yaw)*accel_y_ref_heading_frame;

      double dt = (current_time - prev_update_time_).Double();
      prev_update_time_ = current_time;

      prev_x_vel_ += accel_x_ref * dt;
      prev_y_vel_ += accel_y_ref * dt;

      //math::Vector3 linear_vel2 = parent_->GetLink("base_link")->GetRelativeLinearVel();
      //parent_->GetLink("base_link")->SetLinearVel(math::Vector3(prev_x_vel_, prev_y_vel_, linear_vel2.z));

      math::Vector3 linear_vel_world = link_->GetWorldLinearVel();
      math::Vector3 angular_vel_world = link_->GetWorldAngularVel();

      // Set our desired model velocities (based on the acceleration references)
      linear_vel_world.x = prev_x_vel_;
      linear_vel_world.y = prev_y_vel_;
      angular_vel_world.z = 0; //prev_rot_vel_;   // do not rotate around z-axis

      link_->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0), true);

      double x, y;
      x = 2*cos(0.1 * 2 * M_PI * current_ros_time.toSec());
      y = 2*sin(0.1 * 2 * M_PI * current_ros_time.toSec());

      link_->SetWorldPose(math::Pose(x,y,0,  0,0,0), true, true);

      /*
      math::Vector3 linear_accel = (linear_vel - prev_linear_vel_) / dt;
      prev_linear_vel_ = linear_vel;

      math::Vector3 angular_accel = (angular_vel - prev_angular_vel_) / dt;
      prev_angular_vel_ = angular_vel;

      ROS_INFO("accel (x,y,yaw) = %2.3f\t%2.3f\t%2.3f", linear_accel.x, linear_accel.y, angular_accel.z);*/


#endif
    //parent_->PlaceOnNearestEntityBelow();
    //parent_->SetLinearVel(math::Vector3(
    //      x_ * cosf(yaw) - y_ * sinf(yaw),
    //      y_ * cosf(yaw) + x_ * sinf(yaw),
    //      0));
    //parent_->SetAngularVel(math::Vector3(0, 0, rot_));
  }

  // Finalize the controller
  void GazeboRosPositionControl::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosPositionControl::QuaternionRefCallback(
      const geometry_msgs::Quaternion::ConstPtr& quatMsg)
  {
      boost::mutex::scoped_lock scoped_lock(lock);
      attitudeReference_.setX(quatMsg->x);
      attitudeReference_.setY(quatMsg->y);
      attitudeReference_.setZ(quatMsg->z);
      attitudeReference_.setW(quatMsg->w);

      last_cmd_update_time_ = ros::Time::now();
  }

  void GazeboRosPositionControl::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPositionControl)
}

