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

#include <gtest/gtest.h>
#include <ignition/common/Console.hh>
#include "Manager.hh"

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunEmptyConfig)
{
  ignition::launch::Manager mgr;

  // Expect false because no config file was set.
  EXPECT_FALSE(mgr.RunConfig(""));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, MissingIgnition)
{
  std::string config =
    "<executable name='gazebo'>"
    "  <command>ign-gazebo-server</command>"
    "</executable>";

  ignition::launch::Manager mgr;

  // Stop the manager after a short pause.
  EXPECT_FALSE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunBadXml)
{
  std::string config =
    "<ignition version='1.0'>"
    " executable></executable"
    "</ignition>";

  ignition::launch::Manager mgr;

  // Stop the manager after a short pause.
  EXPECT_FALSE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunLs)
{
  std::string cmd;

#ifdef _MSCV_VER
  cmd = "dir";
#else
  cmd = "ls";
#endif

  std::string config =
    "<ignition version='1.0'>"
    "  <executable name='ls'>"
    "    <command>" + cmd + "</command>"
    "  </executable>"
    "</ignition>";

  ignition::launch::Manager mgr;

  // We should be able to run the ls command. This does not check the
  // output.
  EXPECT_TRUE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
