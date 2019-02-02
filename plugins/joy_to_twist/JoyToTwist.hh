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
#ifndef IGNITION_LAUNCH_JOYTOTWIST_COMPRESS_PLUGIN_HH_
#define IGNITION_LAUNCH_JOYTOTWIST_COMPRESS_PLUGIN_HH_

#include <thread>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>

#include "ignition/launch/Plugin.hh"

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

using namespace ignition::launch;

namespace ignition
{
  class JoyToTwist : public ignition::launch::Plugin
  {
    public: JoyToTwist();
    public: virtual ~JoyToTwist();
    public: virtual void Load(std::map<std::string, std::string> _params) override final;
    public: virtual void Shutdown() override final;
    private: void Run();

    private: void OnJoy(const ignition::msgs::Joy &_msg);

    private: int enableButton;
    private: int enableTurboButton;
    private: ignition::math::Vector3d axisLinear;
    private: ignition::math::Vector3d scaleLinear;
    private: ignition::math::Vector3d scaleLinearTurbo;
    private: ignition::math::Vector3d axisAngular;
    private: ignition::math::Vector3d scaleAngular;
    private: ignition::math::Vector3d scaleAngularTurbo;
    private: bool sentDisableMsg;

    private: bool run = true;
    private: std::thread *joyThread = nullptr;

    private: ignition::transport::Node node;
    private: ignition::transport::Node::Publisher cmdVelPub;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(JoyToTwist, ignition::launch::Plugin)

#endif
