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


#include "amr_joy_setpoint_interface/joy_setpoint.h"

#include <mav_msgs/default_topics.h>

Joy_setpoint::Joy_setpoint() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  setpoint_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("joy_setpoint", 10);

  pnh.param("axis_z_", axes_.pt_z, 3);
  pnh.param("axis_y_", axes_.pt_y, 0);
  pnh.param("axis_x_", axes_.pt_x, 1);

  pnh.param("axis_z_direction", axes_.z_direction, 1);
  pnh.param("axis_y_direction", axes_.y_direction, -1);
  pnh.param("axis_x_direction", axes_.x_direction, 1);

  current_pt_x = 0;
  current_pt_y = 0;
  current_pt_z = 1.5;

  pnh.param("max_pt_x", max_.pt_x, 10.0);  // [m]
  pnh.param("max_pt_y", max_.pt_y, 10.0);  // [m]
  pnh.param("max_pt_z", max_.pt_z, 6.0);   // [m]

  pnh.param("button_button0_", buttons_.button0, 3);
  pnh.param("button_button1_", buttons_.button1, 4);
  pnh.param("button_button2_", buttons_.button2, 5);
  pnh.param("button_button3_", buttons_.button3, 10);
  pnh.param("button_button4_", buttons_.button4, 7);
  pnh.param("button_button5_", buttons_.button5, 8);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy_setpoint::JoyCallback, this);
}


void Joy_setpoint::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  if(msg->axes[axes_.pt_x] * axes_.x_direction > 0)
  {
    current_pt_x = (current_pt_x + 0.1 > max_.pt_x)?max_.pt_x:current_pt_x + 0.1;
  }
  else if(msg->axes[axes_.pt_x] * axes_.x_direction < 0)
  {
    current_pt_x = (current_pt_x - 0.1 < -max_.pt_x)?-max_.pt_x:current_pt_x - 0.1;
  }
  
  if(msg->axes[axes_.pt_y] * axes_.y_direction > 0)
  {
    current_pt_y = (current_pt_y + 0.1 > max_.pt_y)?max_.pt_y:current_pt_y + 0.1;
  }
  else if(msg->axes[axes_.pt_y] * axes_.y_direction < 0)
  {
    current_pt_y = (current_pt_y - 0.1 < -max_.pt_y)?-max_.pt_y:current_pt_y - 0.1;
  }

  if(msg->axes[axes_.pt_z] * axes_.z_direction > 0)
  {
    current_pt_z = (current_pt_z + 0.1 > max_.pt_z)?max_.pt_z:current_pt_z + 0.1;
  }
  else if(msg->axes[axes_.pt_z] * axes_.z_direction < 0)
  {
    current_pt_z = (current_pt_z - 0.1 < 1.5)?1.5:current_pt_z - 0.1;
  }
  setpoint_.point.x = current_pt_x;
  setpoint_.point.y = current_pt_y;
  setpoint_.point.z = current_pt_z;
  ros::Time update_time = ros::Time::now();
  setpoint_.header.stamp = update_time;
  setpoint_.header.frame_id = "world";
  Publish();
}

void Joy_setpoint::Publish() {
  setpoint_pub_.publish(setpoint_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_setpoiny_interface");
  Joy_setpoint joy;
  ros::spin();

  return 0;
}
