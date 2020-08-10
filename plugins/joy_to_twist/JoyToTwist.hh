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

#include <string>
#include <thread>
#include <ignition/launch/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
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
    /// <plugin name="ignition::launch::JoyToTwist"
    ///         filename=ignition-launch-joytotwist0">
    ///   <!-- Incoming topic that publishes
    ///         ignition::msgs::Joystick messages -->
    ///   <input_topic>/joy</input_topic>
    ///
    ///   <!-- Outgoing topic that publishes ignition::msgs::Twist messages -->
    ///   <output_topic>/model/vehicle_blue/cmd_vel</output_topic>
    ///
    ///   <!-- Button which must be pressed to enable publishing,
    ///        defaults to 0 -->
    ///   <enable_button>0</enable_button>
    ///
    ///   <!-- Joystick axis for linear control of XYZ, defaults to [1 0 0]-->
    ///   <axis_linear>1 0 0</axis_linear>
    ///
    ///   <!-- Scale for linear XYZ, defaults to [0.5 0 0] -->
    ///   <scale_linear>2 0 0</scale_linear>
    ///
    ///   <!-- Scale for linear XYZ with turbo mode, defaults [0.5 0 0] -->
    ///   <scale_linear_turbo>5 0 0</scale_linear_turbo>
    ///
    ///   <!-- Joystick axis for angular control, defaults to [0 0 0] -->
    ///   <axis_angular>0 0 1</axis_angular>
    ///
    ///   <!-- Scale for angular control, defaults to [0 0 0.5]-->
    ///   <scale_angular>0 0 0.5</scale_angular>
    ///
    ///   <!-- Scale for angular control with turbo mode,
    ///        defaults [0 0 0.5] -->
    ///   <scale_angular_turbo>0 0 0.5</scale_angular_turbo>
    ///
    ///   <!-- Button which must be pressed to enable turbo,
    ///        defaults to invalid (-1) -->
    ///   <enable_turbo_button>4</enable_turbo_button>
    /// </plugin>
    class JoyToTwist : public ignition::launch::Plugin
    {
      /// \brief Constructor
      public: JoyToTwist();

      /// \brief Destructor
      public: virtual ~JoyToTwist();

      // Documentation inherited
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Run the plugin
      private: void Run();

      /// \brief Handle joystick messages.
      /// \param[in] _msg Joystick message.
      private: void OnJoy(const ignition::msgs::Joy &_msg);

      private: int enableButton = 0;
      private: int enableTurboButton = -1;
      private: ignition::math::Vector3d axisLinear{1.0, 0.0, 0.0};
      private: ignition::math::Vector3d scaleLinear{0.5, 0.0, 0.0};
      private: ignition::math::Vector3d scaleLinearTurbo{0.5, 0.0, 0.0};

      private: ignition::math::Vector3d axisAngular{0.0, 0.0, 0.0};
      private: ignition::math::Vector3d scaleAngular{0.0, 0.0, 0.5};
      private: ignition::math::Vector3d scaleAngularTurbo{0.0, 0.0, 0.5};
      private: bool sentDisableMsg = false;

      private: bool running = false;
      private: std::thread *joyThread = nullptr;

      private: ignition::transport::Node node;
      private: ignition::transport::Node::Publisher cmdVelPub;

      private: std::string inputTopic = "/joy";
      private: std::string outputTopic = "/cmd_vel";
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::JoyToTwist, ignition::launch::Plugin)

#endif
