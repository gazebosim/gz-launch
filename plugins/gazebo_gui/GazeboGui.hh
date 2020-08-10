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
#ifndef IGNITION_LAUNCH_GAZEBOGUI_HH_
#define IGNITION_LAUNCH_GAZEBOGUI_HH_

#include <memory>
#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/launch/Plugin.hh>

namespace ignition
{
  namespace launch
  {
    /// \brief Runs the Ignition Gazebo GUI.
    ///
    /// The plugin ignores GUI configuration coming from the SDF
    /// world file or saved in ~/.ignition/gazebo/gui.config. Instead,
    /// it loads a GUI config from ~/.ignition/launch/gui.config.
    /// If that file doesn't exist, it will be created and populated
    /// with default values. Delete that file to restore default values.
    ///
    /// # Example usage
    ///
    /// <!-- The GUI wants to be in its own process, so wrap the plugin -->
    /// <executable_wrapper>
    ///   <plugin name="ignition::launch::GazeboGui"
    ///           filename="ignition-launch-gazebogui">
    ///
    ///   <!-- Elements parsed by ign-launch -->
    ///
    ///   <!-- Set window title. Defaults to "Gazebo" -->
    ///   <window_title>Custom window title</window_title>
    ///
    ///   <!-- Set window icon.
    ///        This can be a full path to a local file, or a path to a resource
    ///        file prefixed by ":".
    ///        Defaults to the Ignition logo. -->
    ///   <window_icon>full/path/to/window/icon.png</window_icon>
    ///
    ///   <!-- Add Ignition GUI plugins here -->
    ///
    ///   </plugin>
    /// </executable_wrapper>
    class GazeboGui : public ignition::launch::Plugin
    {
      /// \brief Constructor.
      public: GazeboGui();

      /// \brief Destructor.
      public: virtual ~GazeboGui();

      // Documentation inherited.
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboGui, ignition::launch::Plugin)

#endif
