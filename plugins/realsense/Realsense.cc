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

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs.hh>

#include "Realsense.hh"

using namespace ignition::launch;

/////////////////////////////////////////////////
Realsense::Realsense()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
Realsense::~Realsense()
{
}

/////////////////////////////////////////////////
bool Realsense::Load(const tinyxml2::XMLElement * /*_elem*/)
{
  rs2::device_list deviceList = this->rs2Context.query_devices();

  if (deviceList.size() == 0)
  {
    std::cerr << "No RealSense device found.\n";
    return false;
  }
  else
  {
    for (size_t deviceCount = 0; deviceCount < deviceList.size(); ++deviceCount)
    {
      rs2::device device;
      try
      {
        device = deviceList[deviceCount];
      }
      catch(const std::exception &_ex)
      {
        std::cerr << "Device " << deviceCount + 1 << "/"
          << deviceList.size() << "failed with exception["
          << _ex.what() << "]\n";
        continue;
      }

      std::string serialNumber = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "Serial Number[" << serialNumber << "]\n";

      std::string physicalId = device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
      std::cout << "Physical ID[" << physicalId << "]\n";

      std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
      std::cout << "Naem[" << name << "]\n";

    }
  }


/*
  std::string cameraName = this->rs2Device.get_info(RS2_CAMERA_INFO_NAME);

  std::cout << "Camera name[" << cameraName << "]\n";
  std::cout << "1\n";
  rs2::pipeline p;
  std::cout << "2\n";
  p.start();
  std::cout << "3\n";

  // Block program until frames arrive
  rs2::frameset frames = p.wait_for_frames();
  std::cout << "4\n";

  // Try to get a frame of a depth image
  rs2::depth_frame depth = frames.get_depth_frame();
  std::cout << "5\n";

  // Get the depth frame's dimensions
  float width = depth.get_width();
  float height = depth.get_height();
  std::cout << "6\n";

  // Query the distance from the camera to the object in the center of the image
  float dist_to_center = depth.get_distance(width / 2, height / 2);
  std::cout << "7\n";

  std::cout << "The camera is facing an object " << dist_to_center << " meters away \n";
  std::cout << "8\n";
*/
  return true;
}
