/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "Manager.hh"

class LaunchPluginDeathTest: public ::testing::Test
{
  protected: void SetUp()
  {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  }

  protected: std::string getConfig(const std::string &_pluginName)
  {
    return
      "<ignition version='1.0'>"
      "  <plugin name='"  + _pluginName + "'"
      "          filename='" + std::string(bad_plugins_LIB) + "'>"
      "  </plugin>"
      "</ignition>";
  }

  protected: ignition::launch::Manager mgr;
};


TEST_F(LaunchPluginDeathTest, SegfaultOnLoad)
{
  auto config = getConfig("SegfaultOnLoad");

  ASSERT_EXIT(mgr.RunConfig(config),
      ::testing::KilledBySignal(SIGSEGV),
      "");
}

