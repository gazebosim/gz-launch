/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "JoyToTwist.hh"

using namespace ignition;

/////////////////////////////////////////////////
JoyToTwist::JoyToTwist()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
JoyToTwist::~JoyToTwist()
{
}

/////////////////////////////////////////////////
void JoyToTwist::Shutdown()
{
}

/////////////////////////////////////////////////
void JoyToTwist::Load(std::map<std::string, std::string> /*_params*/)
{
  std::string twistTopic = "/model/vehicle_blue/cmd_vel";
  this->cmdVelPub = this->node.Advertise<ignition::msgs::Twist>(twistTopic);

  std::string joyTopic = "/joy";
  this->node.Subscribe("/joy", &JoyToTwist::OnJoy, this);

  this->enableButton = 0;
  this->enableTurboButton = -1;

  this->axisLinear  = ignition::math::Vector3d::UnitX;
  this->scaleLinear = ignition::math::Vector3d(2.0, 0, 0);
  this->scaleLinearTurbo = ignition::math::Vector3d(5.0, 0, 0);

  this->axisAngular = ignition::math::Vector3d::Zero;
  this->scaleAngular = ignition::math::Vector3d(0, 0, 2);
  this->scaleAngularTurbo = ignition::math::Vector3d(0, 0, 5);
  this->sentDisableMsg = false;
}

//////////////////////////////////////////////////
void JoyToTwist::OnJoy(const ignition::msgs::Joy &_msg)
{
  ignition::msgs::Twist cmdVelMsg;

  // Turbo mode
  if (enableTurboButton >= 0 && _msg.buttons(enableTurboButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(axisLinear.X()) * scaleLinearTurbo.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(axisLinear.Y()) * scaleLinearTurbo.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(axisLinear.Z()) * scaleLinearTurbo.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(axisAngular.X()) * scaleAngularTurbo.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(axisAngular.Y()) * scaleAngularTurbo.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(axisAngular.Z()) * scaleAngularTurbo.Z());

    this->cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  // Normal mode
  else if (_msg.buttons(enableButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(axisLinear.X()) * scaleLinear.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(axisLinear.Y()) * scaleLinear.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(axisLinear.Z()) * scaleLinear.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(axisAngular.X()) * scaleAngular.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(axisAngular.Y()) * scaleAngular.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(axisAngular.Z()) * scaleAngular.Z());

    this->cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion
    // command in order to stop the robot.
    if (!sentDisableMsg)
    {
      this->cmdVelPub.Publish(cmdVelMsg);
      sentDisableMsg = true;
    }
  }
}
