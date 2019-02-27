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
#ifndef IGNITION_LAUNCH_JOYTOTWIST_HH_
#define IGNITION_LAUNCH_JOYTOTWIST_HH_

#include <mutex>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/launch/Plugin.hh>
#include <ignition/transport/Node.hh>

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

namespace ignition
{
  namespace launch
  {
    /// \brief Converts ignition::msgs::Joystick messages to
    /// ignition::msgs::Twist.
    ///
    /// # Example usage
    ///
    /// <!-- Inform ignition::Launch about the JoyToTwist plugin -->
    ///  <plugin name="ignition::launch::JoyToTwist"
    ///      filename="libignition-launch-joytotwist0.so">
    //
    /// <!-- Incoming topic that publishes ignition::msgs::Joystick messages -->
    ///   <input_topic>/joy</input_topic>
    //
    /// <!-- Outgoing topic that publishes ignition::msgs::Twist messages -->
    ///   <output_topic>/model/vehicle_green/cmd_vel</output_topic>
    /// </plugin>
    class JoyToTwist : public ignition::launch::Plugin
    {
      /// \brief Constructor
      public: JoyToTwist();

      /// \brief Destructor
      public: virtual ~JoyToTwist();

      // Documentation inherited
      public: virtual void Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Run the plugin
      private: void Run();

      /// \brief Handle joystick messages.
      /// \param[in] _msg Joystick message.
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

      private: bool running = false;
      private: std::thread *joyThread = nullptr;

      private: ignition::transport::Node node;
      private: ignition::transport::Node::Publisher cmdVelPub;

      private: std::string inputTopic = "/joy";
      private: std::string outputTopic = "/cmd_vel";
      private: std::mutex mutex;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::JoyToTwist, ignition::launch::Plugin)

#endif
