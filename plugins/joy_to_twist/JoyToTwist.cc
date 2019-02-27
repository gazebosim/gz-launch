/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "JoyToTwist.hh"

using namespace ignition::launch;

/////////////////////////////////////////////////
JoyToTwist::JoyToTwist()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
JoyToTwist::~JoyToTwist()
{
  this->node.Unsubscribe(this->inputTopic);
  this->running = false;
}

/////////////////////////////////////////////////
void JoyToTwist::Load(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *elem;

  elem = _elem->FirstChildElement("output_topic");
  if (elem)
    this->outputTopic = elem->GetText();

  elem = _elem->FirstChildElement("input_topic");
  if (elem)
    this->inputTopic = elem->GetText();

  this->cmdVelPub = this->node.Advertise<ignition::msgs::Twist>(
      this->outputTopic);

  this->enableButton = 0;
  this->enableTurboButton = -1;

  this->axisLinear  = ignition::math::Vector3d::UnitX;
  this->scaleLinear = ignition::math::Vector3d(2.0, 0, 0);
  this->scaleLinearTurbo = ignition::math::Vector3d(5.0, 0, 0);

  this->axisAngular = ignition::math::Vector3d::Zero;
  this->scaleAngular = ignition::math::Vector3d(0, 0, 2);
  this->scaleAngularTurbo = ignition::math::Vector3d(0, 0, 5);
  this->sentDisableMsg = false;

  igndbg << "Loaded JoyToTwist plugin with the following parameters:\n"
    << "  input_topic: " << this->inputTopic << std::endl
    << "  output_topic: " << this->outputTopic << std::endl;
  this->running = true;
  this->node.Subscribe(this->inputTopic, &JoyToTwist::OnJoy, this);
}

//////////////////////////////////////////////////
void JoyToTwist::OnJoy(const ignition::msgs::Joy &_msg)
{
  if (!this->running)
    return;

  ignition::msgs::Twist cmdVelMsg;
  // Turbo mode
  if (this->enableTurboButton >= 0 && _msg.buttons(this->enableTurboButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(this->axisLinear.X()) * this->scaleLinearTurbo.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(this->axisLinear.Y()) * this->scaleLinearTurbo.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(this->axisLinear.Z()) * this->scaleLinearTurbo.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(this->axisAngular.X()) * this->scaleAngularTurbo.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(this->axisAngular.Y()) * this->scaleAngularTurbo.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(this->axisAngular.Z()) * this->scaleAngularTurbo.Z());

    this->cmdVelPub.Publish(cmdVelMsg);
    this->sentDisableMsg = false;
  }
  // Normal mode
  else if (_msg.buttons(this->enableButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(this->axisLinear.X()) * this->scaleLinear.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(this->axisLinear.Y()) * this->scaleLinear.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(this->axisLinear.Z()) * this->scaleLinear.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(this->axisAngular.X()) * this->scaleAngular.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(this->axisAngular.Y()) * this->scaleAngular.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(this->axisAngular.Z()) * this->scaleAngular.Z());

    this->cmdVelPub.Publish(cmdVelMsg);
    this->sentDisableMsg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion
    // command in order to stop the robot.
    if (!this->sentDisableMsg)
    {
      this->cmdVelPub.Publish(cmdVelMsg);
      this->sentDisableMsg = true;
    }
  }
}
