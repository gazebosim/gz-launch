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
#ifndef IGNITION_LAUNCH_JOYSTICK_COMPRESS_PLUGIN_HH_
#define IGNITION_LAUNCH_JOYSTICK_COMPRESS_PLUGIN_HH_

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
  class Joystick : public ignition::launch::Plugin
  {
    public: Joystick();
    public: virtual ~Joystick();
    public: virtual void Load(const tinyxml2::XMLElement *_elem) override final;
    private: void Run();

    private: bool run = true;
    private: int joyFd = -1;
    private: bool stickyButtons = false;
    private: float unscaledDeadzone = 0.05;
    private: float axisScale = 0.0;
    private: float interval = 1.0f;
    private: float accumulationInterval = 0.001f;
    private: std::thread *joyThread = nullptr;

    private: ignition::transport::Node node;
    private: ignition::transport::Node::Publisher pub;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(Joystick, ignition::launch::Plugin)

#endif
