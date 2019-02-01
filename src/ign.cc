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

#include <string.h>

#include <iostream>

#include <ignition/common/Console.hh>

#include "ign.hh"
#include "ignition/launch/config.hh"
#include "ignition/launch/Manager.hh"

//////////////////////////////////////////////////
extern "C" IGNITION_LAUNCH_VISIBLE char *ignitionVersion()
{
  return strdup(IGNITION_LAUNCH_VERSION_FULL);
}

//////////////////////////////////////////////////
extern "C" IGNITION_LAUNCH_VISIBLE void cmdVerbosity(const char *_verbosity)
{
  ignition::common::Console::SetVerbosity(std::atoi(_verbosity));
}

//////////////////////////////////////////////////
extern "C" IGNITION_LAUNCH_VISIBLE int run(const char *_config)
{
  ignition::launch::Manager mgr;
  return mgr.RunConfig(_config);
}
