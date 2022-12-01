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
#ifndef IGNITION_LAUNCH_GAZEBOSERVER_HH_
#define IGNITION_LAUNCH_GAZEBOSERVER_HH_

#include <memory>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Server.hh>
#include "ignition/launch/Plugin.hh"

namespace ignition
{
  namespace launch
  {
    /// \brief Runs the Ignition Gazebo server.
    ///
    /// # Example usage
    /// <!-- Run the gazebo server with a set of plugins -->
    /// <plugin name="ignition::launch::GazeboServer"
    ///         filename="ignition-launch-gazebo">
    ///   <!-- The SDF file to run -->
    ///   <world_file>diff_drive.sdf</world_file>
    ///
    ///   <!-- The physics system -->
    ///   <plugin entity_name="<%= worldName %>"
    ///           entity_type="world"
    ///           filename="ignition-gazebo-physics-system"
    ///           name="ignition::gazebo::systems::v0::Physics">
    ///   </plugin>
    ///
    ///   <!-- Specify any other ignition gazebo plugins here. -->
    /// </plugin>
    class GazeboServer : public ignition::launch::Plugin
    {
      /// \brief Constructor.
      public: GazeboServer();

      /// \brief Destructor.
      public: virtual ~GazeboServer() = default;

      // Documentation inherited.
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Private data pointer
      private: std::unique_ptr<gazebo::Server> server;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboServer, ignition::launch::Plugin)

#endif
