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

#include <regex>

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
  this->LoadDevice(this->rs2Context.query_devices());
  std::function<void(rs2::event_information&)> changeDeviceCallbackFunction =
    [this](rs2::event_information &_info){this->ChangeDeviceCallback(_info);};

  this->rs2Context.set_devices_changed_callback(changeDeviceCallbackFunction);

  this->StartDevice();

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

//////////////////////////////////////////////////
bool Realsense::LoadDevice(const rs2::device_list &_list)
{
  if (_list.size() == 0)
  {
    std::cerr << "No RealSense device found.\n";
    return false;
  }
  else
  {
    for (size_t deviceCount = 0; deviceCount < _list.size(); ++deviceCount)
    {
      rs2::device dev;
      try
      {
        dev = _list[deviceCount];
      }
      catch(const std::exception &_ex)
      {
        std::cerr << "Device " << deviceCount + 1 << "/"
          << _list.size() << "failed with exception["
          << _ex.what() << "]\n";
        continue;
      }

      std::string serialNum = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "Serial Number[" << serialNum << "]\n";

      std::string physicalId = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
      std::cout << "Physical ID[" << physicalId << "]\n";

      std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
      std::cout << "Name[" << name << "]\n";

      std::string portId = this->ParseUsbPort(physicalId);

      if (portId.empty())
      {
        ignerr << "Error extracting usb port from device with physical ID["
          << physicalId << "]\n";
      }
      else
      {
        igndbg << "Realsense device with port number["
          << portId << "] was found.\n";
      }

      if (!serialNum.empty() && !portId.empty())
      {
        this->rs2Device = dev;
        this->rs2SerialNumber= serialNum;
        break;
      }
    }
  }

  return true;
}

//////////////////////////////////////////////////
void Realsense::ChangeDeviceCallback(const rs2::event_information &_info)
{
  if (_info.was_removed(this->rs2Device))
  {
    ignerr << "The device has been disconnected!\n";
    this->rs2Device = rs2::device();
  }

  if (!this->rs2Device)
  {
    rs2::device_list newDevices = _info.get_new_devices();

    if (newDevices.size() > 0)
    {
      igndbg << "Checking for new devices.\n";
      this->LoadDevice(newDevices);
      this->StartDevice();
    }
  }
}

//////////////////////////////////////////////////
std::string Realsense::ParseUsbPort(const std::string &line) const
{
  std::regex portRegex(
      "(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*",
      std::regex_constants::ECMAScript);

  std::string portId;
  std::smatch baseMatch;

  bool found = std::regex_match(line, baseMatch, portRegex);

  if (found)
  {
    portId = baseMatch[1].str();

    if (baseMatch[2].str().empty())
    {
      std::regex endRegex = std::regex(".+(-[0-9]+$)",
          std::regex_constants::ECMAScript);

      bool foundEnd = std::regex_match(portId, baseMatch, endRegex);

      if (foundEnd)
      {
        portId = portId.substr(0, portId.size() - baseMatch[1].str().size());
      }
    }
  }

  return portId;
}

//////////////////////////////////////////////////
void Realsense::StartDevice()
{
  if (!this->rs2Device)
    return;

  std::string pidStr(this->rs2Device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));

  uint16_t pid = std::stoi(pidStr, 0, 16);

  switch(pid)
  {
  case SR300_PID:
  case SR300v2_PID:
  case RS400_PID:
  case RS405_PID:
  case RS410_PID:
  case RS460_PID:
  case RS415_PID:
  case RS420_PID:
  case RS420_MM_PID:
  case RS430_PID:
  case RS430_MM_PID:
  case RS430_MM_RGB_PID:
  case RS435_RGB_PID:
  case RS435i_RGB_PID:
  case RS_USB2_PID:
  case RS_L515_PID:
    {
      this->realSenseCamera = std::unique_ptr<BaseRealSenseCamera>(
          new BaseRealSenseCamera(this->rs2Device, this->rs2SerialNumber));
      break;
    }
  case RS_T265_PID:
    // todo
    ignerr << "RealSense T265 not supported.\n";
    break;
  default:
    ignerr <<
      "Unsupported Realsense with Product ID: 0x" << pid_str << std::endl;
    break;
  }
}
