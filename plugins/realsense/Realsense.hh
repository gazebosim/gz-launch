/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_LAUNCH_REALSENSE_HH_
#define IGNITION_LAUNCH_REALSENSE_HH_

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <thread>
#include <ignition/launch/Plugin.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Util.hh>

namespace ignition
{
  namespace launch
  {
    /// \brief Reads from a Realsense
    ///
    /// # Example usage
    ///
    class Realsense : public ignition::launch::Plugin
    {
      /// \brief Constructor
      public: Realsense();

      /// \brief Destructor
      public: virtual ~Realsense();

      // Documentation inherited
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      private: ignition::transport::Node node;

      private: rs2::context rs2Context;
      private: rs2::device rs2Device;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::Realsense, ignition::launch::Plugin)

#endif
