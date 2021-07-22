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
#ifndef IGNITION_LAUNCH_MANAGER_HH_
#define IGNITION_LAUNCH_MANAGER_HH_

#include <memory>
#include <string>

#include <ignition/common/SuppressWarning.hh>

#include <ignition/launch/Export.hh>

namespace ignition
{
  namespace launch
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_LAUNCH_VERSION_NAMESPACE {
    // Forward declaration of the private data class
    class ManagerPrivate;

    /// \brief Class for starting and managing programs and plugins.
    class IGNITION_LAUNCH_VISIBLE Manager
    {
      /// \brief Constructor.
      public: Manager();

      /// \brief Destructor.
      public: ~Manager();

      /// \brief Run the mananger based on a configuration string.
      /// \param[in] _config The XML configuration string.
      /// \return True if the configuration was successfully run.
      public: bool RunConfig(const std::string &_config);

      /// \brief Stop running. This can be used to end a Run().
      /// \return True if running, False if the state was not running.
      public: bool Stop();

      /// \brief Private data pointer.
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      private: std::unique_ptr<ManagerPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
