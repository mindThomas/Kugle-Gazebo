/* Copyright (C) 2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */

/* Inspired from https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/blob/indigo-devel/hector_gazebo_plugins/src/gazebo_ros_force_based_move.cpp */
/*
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

#include <gazebo_ballbot_acceleration_control.h>
#include <cmath>       /* isnan, sqrt */

namespace gazebo 
{

  GazeboRosBallbotAccelerationControl::GazeboRosBallbotAccelerationControl() {}

  GazeboRosBallbotAccelerationControl::~GazeboRosBallbotAccelerationControl() {}

  // Load the controller
  void GazeboRosBallbotAccelerationControl::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

      world_frame_ = "world";
      if (!sdf->HasElement("worldFrame"))
      {
          ROS_WARN("PlanarMovePlugin (ns = %s) missing <worldFrame>, "
                   "defaults to \"%s\"",
                   robot_namespace_.c_str(), world_frame_.c_str());
      }
      else
      {
          world_frame_ = sdf->GetElement("worldFrame")->Get<std::string>();
      }


    torque_yaw_velocity_p_gain_ = 100.0;
    force_x_velocity_p_gain_ = 10000.0;
    force_y_velocity_p_gain_ = 10000.0;
    
    if (sdf->HasElement("yaw_velocity_p_gain"))
      (sdf->GetElement("yaw_velocity_p_gain")->GetValue()->Get(torque_yaw_velocity_p_gain_));

    if (sdf->HasElement("x_velocity_p_gain"))
      (sdf->GetElement("x_velocity_p_gain")->GetValue()->Get(force_x_velocity_p_gain_));

    if (sdf->HasElement("y_velocity_p_gain"))
      (sdf->GetElement("y_velocity_p_gain")->GetValue()->Get(force_y_velocity_p_gain_));
      
    ROS_INFO_STREAM("ForceBasedMove using gains: yaw: " << torque_yaw_velocity_p_gain_ <<
                                                 " x: " << force_x_velocity_p_gain_ <<
                                                 " y: " << force_y_velocity_p_gain_ << "\n");

    robot_base_link_ = "base_link";
    if (!sdf->HasElement("robotBaseLink"))
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseLink>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_link_.c_str());
    } 
    else 
    {
      robot_base_link_ = sdf->GetElement("robotBaseLink")->Get<std::string>();
    }

    ROS_INFO_STREAM("robotBaseLink for force based move plugin: " << robot_base_link_  << "\n");

    this->link_ = parent->GetLink(robot_base_link_);

    if (!this->link_)
    {
      ROS_FATAL_STREAM("Could not find link for base frame: " << robot_base_link_ << "\n");
      return;
    }

      /*robot_link_ = "robot";
      if (!sdf->HasElement("robotLink"))
      {
          ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotLink>, "
                   "defaults to \"%s\"",
                   robot_namespace_.c_str(), robot_link_.c_str());
      }
      else
      {
          robot_link_ = sdf->GetElement("robotLink")->Get<std::string>();
      }

      ROS_INFO_STREAM("robotBaseLink for force based move plugin: " << robot_link_  << "\n");

      this->robotLink_ = link_->GetChildLink(robot_link_);

      if (!this->robotLink_)
      {
          ROS_FATAL_STREAM("Could not find link for robot frame: " << robot_link_ << "\n");
          return;
      }*/

      robot_base_joint_ = "base_link_to_robot";
      if (!sdf->HasElement("robotBaseJoint"))
      {
          ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseJoint>, "
                   "defaults to \"%s\"",
                   robot_namespace_.c_str(), robot_base_joint_.c_str());
      }
      else
      {
          robot_base_joint_ = sdf->GetElement("robotBaseJoint")->Get<std::string>();
      }

      ROS_INFO_STREAM("robotBaseJoint for force based move plugin: " << robot_base_joint_  << "\n");

      this->joint_ = parent->GetJoint(robot_base_joint_);

      if (!this->joint_)
      {
          ROS_WARN_STREAM("Could not find joint for base frame to robot: " << robot_base_joint_ << "\n");
      }


    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

  cmd_timeout_ = 1.0;
  if (!sdf->HasElement("cmdTimeout"))
  {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <cmdTimeout>, "
               "defaults to %f",
               robot_namespace_.c_str(), cmd_timeout_);
  }
  else
  {
      cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();
  }

      this->publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("PlanarMovePlugin Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    }
 
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    attitudeReference_ = tf::Quaternion(0,0,0,1); // set as unit quaternion

    odom_transform_.setIdentity();

#if (GAZEBO_MAJOR_VERSION >= 8)
      prev_update_time_ = parent_->GetWorld()->SimTime();
#else
      prev_update_time_ = parent_->GetWorld()->GetSimTime();
#endif

    last_cmd_update_time_ = ros::Time::now();
    prev_linear_vel_ = math::Vector3::Zero;
    prev_angular_vel_ = math::Vector3::Zero;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    if (publish_odometry_tf_)
      transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosBallbotAccelerationControl::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    joints_pub_ = rosnode_->advertise<sensor_msgs::JointState>(robot_namespace_ + "/joint_states", 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosBallbotAccelerationControl::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosBallbotAccelerationControl::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosBallbotAccelerationControl::UpdateChild()
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
          x_ = 0.0;
          y_ = 0.0;
          rot_ = 0.0;

          attitudeReference_ = tf::Quaternion(0,0,0,1); // set as unit quaternion
          attitudeReference_.setRPY(0.2,0,0);
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

      // Set the roll, pitch and yaw joints
      if (parent_->GetJoint("roll"))
          parent_->GetJoint("roll")->SetPosition(0, roll);

      if (parent_->GetJoint("pitch"))
          parent_->GetJoint("pitch")->SetPosition(0, pitch);

      if (parent_->GetJoint("yaw"))
          parent_->GetJoint("yaw")->SetPosition(0, yaw);

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
    link_->SetForce(math::Vector3(x_, y_, 0.0));

      /*double dt = (current_time - prev_update_time_).Double();
      prev_update_time_ = current_time;

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


    /* Handle Odomotry publish */
    if (odometry_rate_ > 0.0) {
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        publishWorldGroundTruth();
        publishJointStates();
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosBallbotAccelerationControl::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosBallbotAccelerationControl::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;

    last_cmd_update_time_ = ros::Time::now();
  }

  void GazeboRosBallbotAccelerationControl::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

    void GazeboRosBallbotAccelerationControl::publishJointStates(void)
    {
        /* Joint states are eg. used for visualization in RVIZ */

        // Convert attitude reference to roll, pitch and yaw
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(attitudeReference_);
        mat.getEulerYPR(yaw, pitch, roll);

        // Define the joint state
        sensor_msgs::JointState joint_state;

        ros::Time current_time = ros::Time::now();
        joint_state.header.stamp = current_time;

        joint_state.name.push_back("roll");
        joint_state.position.push_back(roll);
        joint_state.effort.push_back(0.0);
        joint_state.velocity.push_back(0.0);

        joint_state.name.push_back("pitch");
        joint_state.position.push_back(pitch);
        joint_state.effort.push_back(0.0);
        joint_state.velocity.push_back(0.0);

        joint_state.name.push_back("yaw");
        joint_state.position.push_back(yaw);
        joint_state.effort.push_back(0.0);
        joint_state.velocity.push_back(0.0);

        joints_pub_.publish(joint_state);
    }

    void GazeboRosBallbotAccelerationControl::publishWorldGroundTruth(void)
    {
        ros::Time current_time = ros::Time::now();
        std::string world_frame = tf::resolve(tf_prefix_, world_frame_);
        std::string base_footprint_frame =
                tf::resolve(tf_prefix_, robot_base_link_);

#if (GAZEBO_MAJOR_VERSION >= 8)

#else
        math::Pose pose = parent_->GetWorldPose();
        math::Vector3 linear_vel = parent_->GetWorldLinearVel();
        math::Vector3 angular_vel = parent_->GetWorldAngularVel();
        float yaw = pose.rot.GetYaw();

        tf::Transform transform;
        transform.setIdentity();
        transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
        transform.setRotation(tf::createQuaternionFromYaw(yaw));

#endif
        if (transform_broadcaster_.get()){
            transform_broadcaster_->sendTransform(
                    tf::StampedTransform(transform.inverse(), current_time, base_footprint_frame, world_frame)
            );
        }
    }

  void GazeboRosBallbotAccelerationControl::publishOdometry(double step_time)
  {
    /* Odometry is based on integration of the relative velocity in forward direction and angular (yaw) velocity */
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_link_);

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.X(), angular_vel.Z(), step_time);

    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.twist.twist.angular.z = angular_vel.Z();
    odom_.twist.twist.linear.x  = linear_vel.X();
#else
    math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, angular_vel.z, step_time);

    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.twist.twist.angular.z = angular_vel.z;
    odom_.twist.twist.linear.x  = linear_vel.x;
#endif

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (transform_broadcaster_.get()){
      transform_broadcaster_->sendTransform(
          tf::StampedTransform(odom_transform_, current_time, odom_frame,
              base_footprint_frame));
    }
    
    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    
#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.pose.covariance[35] = 0.01;
    }else{
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.twist.covariance[35] = 0.01;
    }else{
      odom_.twist.covariance[35] = 100.0;
    }



    odometry_pub_.publish(odom_);
  }


  tf::Transform GazeboRosBallbotAccelerationControl::getTransformForMotion(double linear_vel_x, double angular_vel, double timeSeconds) const
  {
    tf::Transform tmp;
    tmp.setIdentity();


    if (std::abs(angular_vel) < 0.0001) {
      //Drive straight
      tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), 0.0, 0.0));
    } else {
      //Follow circular arc
      double distChange = linear_vel_x * timeSeconds;
      double angleChange = angular_vel * timeSeconds;

      double arcRadius = distChange / angleChange;

      tmp.setOrigin(tf::Vector3(std::sin(angleChange) * arcRadius,
                                arcRadius - std::cos(angleChange) * arcRadius,
                                0.0));
      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }

    return tmp;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosBallbotAccelerationControl)
}

