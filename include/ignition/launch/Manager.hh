/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <map>
#include <ignition/launch/Export.hh>

namespace ignition
{
  namespace launch
  {
    // Forward declaration of the private data class
    class ManagerPrivate;

    /// \brief Class for starting and managing programs and plugins.
    class IGNITION_LAUNCH_VISIBLE Manager
    {
      /// \brief Constructor.
      public: Manager();

      /// \brief Destructor.
      public: virtual ~Manager();

      /// \brief Run programs and plugins.
      /// This function blocks until stopped by the user.
      /// \param[in] _args Command line arguments.
      /// \return True if running terminated successfully.
      public: bool Run(const std::map<std::string, std::string> &_args = {});

      /// \brief Stop running. This can be used to end a Run().
      /// \return True if running, False if the state was not running.
      public: bool Stop();

      /// \brief Private data pointer.
      private: std::unique_ptr<ManagerPrivate> dataPtr;
    };
  }
}
#endif
