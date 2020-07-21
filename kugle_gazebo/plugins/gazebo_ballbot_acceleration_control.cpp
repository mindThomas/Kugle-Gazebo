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

#include <gazebo_ballbot_acceleration_control.h>
#include <cmath>       /* isnan, sqrt */
#include <kugle_msgs/StateEstimate.h>

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
      ROS_INFO("BallbotAccelerationControl missing <robotNamespace>, "
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
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <commandTopic>, "
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
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <odometryTopic>, "
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
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <odometryFrame>, "
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
        ROS_WARN("BallbotAccelerationControl (ns = %s) missing <worldFrame>, "
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
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <contactPointLink>, "
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

      heading_link_ = "heading";
      if (!sdf->HasElement("headingLink"))
      {
          ROS_WARN("BallbotAccelerationControl (ns = %s) missing <headingLink>, "
                   "defaults to \"%s\"",
                   heading_link_.c_str(), heading_link_.c_str());
      }
      else
      {
          heading_link_ = sdf->GetElement("headingLink")->Get<std::string>();
      }

      base_link_ = "base_link";
      if (!sdf->HasElement("baseLink"))
      {
          ROS_WARN("BallbotAccelerationControl (ns = %s) missing <baseLink>, "
                   "defaults to \"%s\"",
                   base_link_.c_str(), base_link_.c_str());
      }
      else
      {
          base_link_ = sdf->GetElement("baseLink")->Get<std::string>();
      }

    odometry_rate_ = 200.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

      state_estimate_topic_ = "StateEstimate";
      if (!sdf->HasElement("stateEstimateTopic"))
      {
          ROS_WARN("BallbotAccelerationControl (ns = %s) missing <stateEstimateTopic>, "
                   "defaults to \"%s\"",
                   robot_namespace_.c_str(), state_estimate_topic_.c_str());
      }
      else
      {
          state_estimate_topic_ = sdf->GetElement("stateEstimateTopic")->Get<std::string>();
      }

  cmd_timeout_ = 1.0;
  if (!sdf->HasElement("cmdTimeout"))
  {
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <cmdTimeout>, "
               "defaults to %f",
               robot_namespace_.c_str(), cmd_timeout_);
  }
  else
  {
      cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();
  }


  acceleration_coefficient_ = 14.5289;
  if (!sdf->HasElement("accelerationCoefficient"))
  {
      ROS_WARN("BallbotAccelerationControl (ns = %s) missing <accelerationCoefficient>, "
               "defaults to %f",
               robot_namespace_.c_str(), acceleration_coefficient_);
  }
  else
  {
      acceleration_coefficient_ = sdf->GetElement("accelerationCoefficient")->Get<double>();
  }

      this->publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("BallbotAccelerationControl Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
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

    prev_x_vel_ = 0;
    prev_y_vel_ = 0;
    prev_yaw_ = 0;
    prev_yaw_for_odometry_ = 0;
    mcu_time_ = 0;
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
      ROS_FATAL_STREAM("BallbotAccelerationControl (ns = " << robot_namespace_
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
      ros::SubscribeOptions::create<geometry_msgs::Quaternion>(command_topic_, 1,
          boost::bind(&GazeboRosBallbotAccelerationControl::QuaternionRefCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    state_estimate_pub_ = rosnode_->advertise<kugle_msgs::StateEstimate>(state_estimate_topic_, 1);
    joints_pub_ = rosnode_->advertise<sensor_msgs::JointState>("/joint_states", 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosBallbotAccelerationControl::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosBallbotAccelerationControl::UpdateChild, this));

  }

  float GazeboRosBallbotAccelerationControl::unwrap(float previous_angle, float new_angle)
  {
      float d = new_angle - previous_angle;
      d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
      return previous_angle + d;
  }

  // Called when simulation initializes
  void GazeboRosBallbotAccelerationControl::Init()
  {

  }

  // Called when Gazebo world/simulation is reset
  void GazeboRosBallbotAccelerationControl::Reset()
  {
      prev_x_vel_ = 0;
      prev_y_vel_ = 0;
      prev_yaw_ = 0;
      prev_yaw_for_odometry_ = 0;
      mcu_time_ = 0;
      alive_ = true;

      attitudeReference_ = tf::Quaternion(0,0,0,1); // set as unit quaternion

      odom_transform_.setIdentity();

#if (GAZEBO_MAJOR_VERSION >= 8)
      prev_update_time_ = parent_->GetWorld()->SimTime();
#else
      prev_update_time_ = parent_->GetWorld()->GetSimTime();
#endif

      last_cmd_update_time_ = ros::Time::now();
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

      // Set the roll, pitch and yaw joints
      if (parent_->GetJoint("roll"))
          parent_->GetJoint("roll")->SetPosition(0, roll);

      if (parent_->GetJoint("pitch"))
          parent_->GetJoint("pitch")->SetPosition(0, pitch);

      if (parent_->GetJoint("yaw")) {
          /*if (abs(yaw) > M_PI/2) {
              if (copysign(1, yaw) != copysign(1, prev_yaw_)) {
                  // sign has suddenly change, probably because of a roll-over
                  if (yaw > 0) yaw -= 2 * M_PI; // yaw has suddenly become positive, subtract 360 degrees
                  else yaw += 2 * M_PI; // yaw has suddenly become negative, add 360 degree
              }
          }*/

          yaw = unwrap(prev_yaw_, yaw);
          parent_->GetJoint("yaw")->SetPosition(0, yaw);
          prev_yaw_ = yaw;
      }

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

#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Vector3d linear_vel_world = link_->WorldLinearVel();
      ignition::math::Vector3d angular_vel_world = link_->WorldAngularVel();

      // Set our desired model velocities (based on the acceleration references)
      linear_vel_world.X() = prev_x_vel_;
      linear_vel_world.Y() = prev_y_vel_;
      angular_vel_world.Z() = 0; //prev_rot_vel_;   // do not rotate around z-axis

      link_->SetWorldTwist(linear_vel_world, ignition::math::Vector3d(0,0,0), true);
#else
      math::Vector3 linear_vel_world = link_->GetWorldLinearVel();
      math::Vector3 angular_vel_world = link_->GetWorldAngularVel();

      // Set our desired model velocities (based on the acceleration references)
      linear_vel_world.x = prev_x_vel_;
      linear_vel_world.y = prev_y_vel_;
      angular_vel_world.z = 0; //prev_rot_vel_;   // do not rotate around z-axis

      link_->SetWorldTwist(linear_vel_world, math::Vector3(0,0,0), true);
#endif

      /*
      math::Vector3 linear_accel = (linear_vel - prev_linear_vel_) / dt;
      prev_linear_vel_ = linear_vel;

      math::Vector3 angular_accel = (angular_vel - prev_angular_vel_) / dt;
      prev_angular_vel_ = angular_vel;

      ROS_INFO("accel (x,y,yaw) = %2.3f\t%2.3f\t%2.3f", linear_accel.x, linear_accel.y, angular_accel.z);*/

    //parent_->PlaceOnNearestEntityBelow();
    //parent_->SetLinearVel(math::Vector3(
    //      x_ * cosf(yaw) - y_ * sinf(yaw),
    //      y_ * cosf(yaw) + x_ * sinf(yaw),
    //      0));
    //parent_->SetAngularVel(math::Vector3(0, 0, rot_));


    /* Handle Odometry publish */
    if (odometry_rate_ > 0.0) {
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometryAndStateEstimate(seconds_since_last_update);
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

  void GazeboRosBallbotAccelerationControl::QuaternionRefCallback(
      const geometry_msgs::Quaternion::ConstPtr& quatMsg)
  {
      boost::mutex::scoped_lock scoped_lock(lock);
      attitudeReference_.setX(quatMsg->x);
      attitudeReference_.setY(quatMsg->y);
      attitudeReference_.setZ(quatMsg->z);
      attitudeReference_.setW(quatMsg->w);

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
        /*std::string base_footprint_frame =
                tf::resolve(tf_prefix_, robot_base_link_);*/
        std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);

#if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Pose3d pose = parent_->WorldPose();
        ignition::math::Vector3d linear_vel = parent_->WorldLinearVel();
        ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();
        float yaw = pose.Rot().Yaw();

        tf::Transform base_link_tf; // base_link frame defined in world frame
        base_link_tf.setIdentity();
        base_link_tf.setOrigin(tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
        base_link_tf.setRotation(tf::createQuaternionFromYaw(yaw));
#else
        math::Pose pose = parent_->GetWorldPose();
        math::Vector3 linear_vel = parent_->GetWorldLinearVel();
        math::Vector3 angular_vel = parent_->GetWorldAngularVel();
        float yaw = pose.rot.GetYaw();

        tf::Transform base_link_tf; // base_link frame defined in world frame
        base_link_tf.setIdentity();
        base_link_tf.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
        base_link_tf.setRotation(tf::createQuaternionFromYaw(yaw));
#endif

        // odom_transform_  defines the base_link frame in odometry frame
        // Compute the transform of world frame defined in odometry frame
        tf::Transform worldCorrectionTransform = odom_transform_ * base_link_tf.inverse();

        if (transform_broadcaster_.get()){
            transform_broadcaster_->sendTransform(
                    tf::StampedTransform(worldCorrectionTransform, current_time, odom_frame, world_frame)
            );
        }
    }

  void GazeboRosBallbotAccelerationControl::publishOdometryAndStateEstimate(double step_time)
  {
    /* Odometry is based on integration of the linear (translational) velocity */
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string contact_point_frame = tf::resolve(tf_prefix_, contact_point_link_);
    std::string heading_frame = tf::resolve(tf_prefix_, heading_link_);
    std::string base_link_frame = tf::resolve(tf_prefix_, base_link_);
    tf::Quaternion q = attitudeReference_;

#if (GAZEBO_MAJOR_VERSION >= 8)
/* We want to create ballbot odometry which is in the heading frame */
      // First we get the world velocity
      ignition::math::Vector3d linear_vel = parent_->WorldLinearVel();

      // Then we determine the yaw/heading and rotate the linear velocity to get it in the heading frame
      tfScalar yaw, pitch, roll;
      tf::Matrix3x3 mat(q);
      mat.getEulerYPR(yaw, pitch, roll);
      tf::Vector3 linear_vel_base_link = mat.transpose() * tf::Vector3(linear_vel.X(),linear_vel.Y(),0); // compute linear velocity in base_link frame
      tf::Quaternion q_heading = tf::createQuaternionFromYaw(yaw);
      mat.setRotation(q_heading);
      tf::Vector3 linear_vel_heading = mat.transpose() * tf::Vector3(linear_vel.X(),linear_vel.Y(),0);

      // Determine angular velocity around z-axis based on discrete differentiation of yaw
      double angular_velocity = (yaw - prev_yaw_for_odometry_) / step_time;
      prev_yaw_for_odometry_ = yaw;

      // Compute dq
      tf::Quaternion q_omega(0, 0, angular_velocity, 0); // x,y,z,w - only set z angular velocity to yaw turning rate
      tf::Quaternion dq = q_omega * q * 0.5; // dq = 1/2 * Phi(q_omega) * q

      // Create delta transforms
      /*tf::Transform deltaRot;
      deltaRot.setIdentity();
      deltaRot.setRotation(tf::createQuaternionFromYaw(angular_velocity*step_time));*/
      tf::Transform deltaPos;
      deltaPos.setIdentity();
      deltaPos.setOrigin(tf::Vector3(static_cast<double>(linear_vel.X()*step_time), static_cast<double>(linear_vel.Y()*step_time), 0.0));
#else

    /* We want to create ballbot odometry which is in the heading frame */
    // First we get the world velocity
    math::Vector3 linear_vel = parent_->GetWorldLinearVel();

    // Then we determine the yaw/heading and rotate the linear velocity to get it in the heading frame
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    tf::Vector3 linear_vel_base_link = mat.transpose() * tf::Vector3(linear_vel.x,linear_vel.y,0); // compute linear velocity in base_link frame
    tf::Quaternion q_heading = tf::createQuaternionFromYaw(yaw);
    mat.setRotation(q_heading);
    tf::Vector3 linear_vel_heading = mat.transpose() * tf::Vector3(linear_vel.x,linear_vel.y,0);

    // Determine angular velocity around z-axis based on discrete differentiation of yaw
    double angular_velocity = (yaw - prev_yaw_for_odometry_) / step_time;
    prev_yaw_for_odometry_ = yaw;

    // Compute dq
    tf::Quaternion q_omega(0, 0, angular_velocity, 0); // x,y,z,w - only set z angular velocity to yaw turning rate
    tf::Quaternion dq = q_omega * q * 0.5; // dq = 1/2 * Phi(q_omega) * q

    // Create delta transforms
    /*tf::Transform deltaRot;
    deltaRot.setIdentity();
    deltaRot.setRotation(tf::createQuaternionFromYaw(angular_velocity*step_time));*/
    tf::Transform deltaPos;
    deltaPos.setIdentity();
    deltaPos.setOrigin(tf::Vector3(static_cast<double>(linear_vel.x*step_time), static_cast<double>(linear_vel.y*step_time), 0.0));
#endif

    // Update odometry transform - odometry should not include rotation here, as this is given from the internal robot joints due to the way this Gazebo simulation model was made
    odom_transform_ = odom_transform_ * deltaPos;

    if (transform_broadcaster_.get()){
          transform_broadcaster_->sendTransform(
                  tf::StampedTransform(odom_transform_, current_time, odom_frame, contact_point_frame));
    }

    // Prepare odometry message
    //   This represents an estimate of a position and velocity in free space.
    //   The pose in this message should be specified in the coordinate frame given by header.frame_id
    //   The twist in this message should be specified in the coordinate frame given by the child_frame_id
    //tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.pose.pose.position.x = odom_transform_.getOrigin().x();  // inertial frame position
    odom_.pose.pose.position.y = odom_transform_.getOrigin().y();
    odom_.pose.pose.position.z = odom_transform_.getOrigin().z();
    odom_.pose.pose.orientation.w = q.w();
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();

    odom_.twist.twist.angular.x = 0;
    odom_.twist.twist.angular.y = 0;
    odom_.twist.twist.angular.z = angular_velocity;
    odom_.twist.twist.linear.x  = linear_vel_base_link.x();  // base link frame velocity
    odom_.twist.twist.linear.y  = linear_vel_base_link.y();
    odom_.twist.twist.linear.z  = linear_vel_base_link.z();


    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_link_frame;

    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;

    odom_.pose.covariance[35] = 0.01; // std::abs(angular_vel.z) < 0.0001
    //odom_.pose.covariance[35] = 100.0;

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = 0.01; // std::abs(angular_vel.z) < 0.0001
    //odom_.twist.covariance[35] = 100.0;

    odometry_pub_.publish(odom_);

    //mcu_time_ += step_time;
    mcu_time_ += 1.0f / odometry_rate_;
    mcu_time_ = roundf(1000*mcu_time_) / 1000;

    /* Send state estimate message */
    kugle_msgs::StateEstimate stateEstimate_msg;
    stateEstimate_msg.receive_time = ros::Time::now();
    stateEstimate_msg.mcu_time = mcu_time_;
    stateEstimate_msg.q.w = q.w();
    stateEstimate_msg.q.x = q.x();
    stateEstimate_msg.q.y = q.y();
    stateEstimate_msg.q.z = q.z();
    stateEstimate_msg.dq.w = dq.w();
    stateEstimate_msg.dq.x = dq.x();
    stateEstimate_msg.dq.y = dq.y();
    stateEstimate_msg.dq.z = dq.z();
    stateEstimate_msg.position[0] = float(odom_transform_.getOrigin().x());
    stateEstimate_msg.position[1] = float(odom_transform_.getOrigin().y());
#if (GAZEBO_MAJOR_VERSION >= 8)
    stateEstimate_msg.velocity[0] = float(linear_vel.X()); // inertial frame velocity
    stateEstimate_msg.velocity[1] = float(linear_vel.Y());
#else
    stateEstimate_msg.velocity[0] = float(linear_vel.x); // inertial frame velocity
    stateEstimate_msg.velocity[1] = float(linear_vel.y);
#endif
    state_estimate_pub_.publish(stateEstimate_msg);
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

