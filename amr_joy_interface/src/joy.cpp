/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "amr_joy_interface/joy.h"

#include <mav_msgs/default_topics.h>

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;

  pnh.param("axis_roll_", axes_.roll, 2);
  pnh.param("axis_pitch_", axes_.pitch, 3);
  pnh.param("axis_yawrate_", axes_.yawrate, 0);
  pnh.param("axis_thrust_", axes_.thrust, 1);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_yawrate", axes_.yawrate_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yawrate", max_.yawrate, 90.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  pnh.param("button_button0_", buttons_.button0, 3);
  pnh.param("button_button1_", buttons_.button1, 4);
  pnh.param("button_button2_", buttons_.button2, 5);
  pnh.param("button_button3_", buttons_.button3, 10);
  pnh.param("button_button4_", buttons_.button4, 7);
  pnh.param("button_button5_", buttons_.button5, 8);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}


void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;

  control_msg_.yaw_rate = msg->axes[axes_.yawrate] * max_.yawrate * axes_.yawrate_direction;

  control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;

  ros::spin();

  return 0;
}
