/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <chrono>
#include <gtest/gtest.h>
#include <ignition/common/Console.hh>
#include "ignition/launch/Manager.hh"

/////////////////////////////////////////////////
/// \brief Test running without a config file
TEST(Ignition_TEST, RunNoConfig)
{
  ignition::launch::Manager mgr;

  // Expect false because no config file was set.
  EXPECT_FALSE(mgr.Run());
  EXPECT_FALSE(mgr.Stop());
}

/////////////////////////////////////////////////
/// \brief Test running with an empty config file
TEST(Ignition_TEST, RunEmptyConfig)
{
  std::string configFilename = "/tmp/test.ign";
  std::fstream configFile(configFilename.c_str(), std::ios::out);
  configFile.close();

  std::map<std::string, std::string> args = {{"config", configFilename}};

  ignition::launch::Manager mgr;
  EXPECT_FALSE(mgr.Run(args));
  EXPECT_FALSE(mgr.Stop());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
