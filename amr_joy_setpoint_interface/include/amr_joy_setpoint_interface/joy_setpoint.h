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


#ifndef AMR_JOY_SETPOINT_INTERFACE_JOY_H_
#define AMR_JOY_SETPOINT_INTERFACE_JOY_H_

#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

struct Axes {
  int pt_x;
  int pt_y;
  int pt_z;
  int x_direction;
  int y_direction;
  int z_direction;
  int extra0;
  int extra1;
};

struct Buttons {
  int button0;
  int button1;
  int button2;
  int button3;
  int button4;
  int button5;
};

struct Max {
  double pt_x;
  double pt_y;
  double pt_z;
};

class Joy_setpoint {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher setpoint_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  Axes axes_;
  Buttons buttons_;

  double current_pt_x;
  double current_pt_y;
  double current_pt_z;

  geometry_msgs::PointStamped setpoint_;
  sensor_msgs::Joy current_joy_;

  Max max_;

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  Joy_setpoint();
};

#endif // AMR_JOY_SETPOINT_INTERFACE_JOY_H_
