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
#ifndef IGNITION_LAUNCH_JOYSTICK_HH_
#define IGNITION_LAUNCH_JOYSTICK_HH_

#include <thread>
#include <ignition/launch/Plugin.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace ignition
{
  namespace launch
  {
    /// \brief Reads from a USB joystick device and outputs
    ///  ignition::msgs::Joystick messages.
    ///
    /// # Example usage
    ///
    /// <!-- Inform ignition::Launch about the JoyToTwist plugin -->
    ///  <plugin name="ignition::launch::Joystick"
    ///      filename="ignition-launch-joystick0">
    ///
    ///    <!-- Joystick device -->
    ///    <device>/dev/input/js0</device>
    ///
    ///    <!-- True enables sticky buttons. -->
    ///    <sticky_buttons>false</sticky_buttons>
    ///
    ///    <!-- Joystick deadzone -->
    ///    <dead_zone>0.05</dead_zone>
    ///
    ///    <!-- Update rate -->
    ///    <rate>60</rate>
    ///    <accumulation_rate>1000</accumulation_rate>
    /// </plugin>

    class Joystick : public ignition::launch::Plugin
    {
      /// \brief Constructor
      public: Joystick();

      /// \brief Destructor
      public: virtual ~Joystick();

      // Documentation inherited
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Run the plugin
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
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::Joystick, ignition::launch::Plugin)

#endif
