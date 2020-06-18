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

const uint16_t SR300_PID        = 0x0aa5; // SR300
const uint16_t SR300v2_PID      = 0x0B48; // SR305
const uint16_t RS400_PID        = 0x0ad1; // PSR
const uint16_t RS410_PID        = 0x0ad2; // ASR
const uint16_t RS415_PID        = 0x0ad3; // ASRC
const uint16_t RS430_PID        = 0x0ad4; // AWG
const uint16_t RS430_MM_PID     = 0x0ad5; // AWGT
const uint16_t RS_USB2_PID      = 0x0ad6; // USB2
const uint16_t RS420_PID        = 0x0af6; // PWG
const uint16_t RS420_MM_PID     = 0x0afe; // PWGT
const uint16_t RS410_MM_PID     = 0x0aff; // ASR
const uint16_t RS400_MM_PID     = 0x0b00; // PSR
const uint16_t RS430_MM_RGB_PID = 0x0b01; // AWGCT
const uint16_t RS460_PID        = 0x0b03; // DS5U
const uint16_t RS435_RGB_PID    = 0x0b07; // AWGC
const uint16_t RS435i_RGB_PID   = 0x0B3A; // AWGC_MM
const uint16_t RS405_PID        = 0x0b0c; // DS5U
const uint16_t RS_T265_PID      = 0x0b37; //
const uint16_t RS_L515_PID      = 0x0B3D; //


const bool ALIGN_DEPTH    = false;
const bool POINTCLOUD     = false;
const bool ALLOW_NO_TEXTURE_POINTS = false;
const bool SYNC_FRAMES    = false;

const bool PUBLISH_TF        = true;
const double TF_PUBLISH_RATE = 0; // Static transform

const int IMAGE_WIDTH     = 640;
const int IMAGE_HEIGHT    = 480;
const int IMAGE_FPS       = 30;

const int IMU_FPS         = 0;


const bool ENABLE_DEPTH   = true;
const bool ENABLE_INFRA1  = true;
const bool ENABLE_INFRA2  = true;
const bool ENABLE_COLOR   = true;
const bool ENABLE_FISHEYE = true;
const bool ENABLE_IMU     = true;
const bool HOLD_BACK_IMU_FOR_FRAMES = false;
const bool PUBLISH_ODOM_TF = true;


const std::string DEFAULT_BASE_FRAME_ID            = "camera_link";
const std::string DEFAULT_ODOM_FRAME_ID            = "odom_frame";
const std::string DEFAULT_DEPTH_FRAME_ID           = "camera_depth_frame";
const std::string DEFAULT_INFRA1_FRAME_ID          = "camera_infra1_frame";
const std::string DEFAULT_INFRA2_FRAME_ID          = "camera_infra2_frame";
const std::string DEFAULT_COLOR_FRAME_ID           = "camera_color_frame";
const std::string DEFAULT_FISHEYE_FRAME_ID         = "camera_fisheye_frame";
const std::string DEFAULT_IMU_FRAME_ID             = "camera_imu_frame";

const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID   = "camera_depth_optical_frame";
const std::string DEFAULT_INFRA1_OPTICAL_FRAME_ID  = "camera_infra1_optical_frame";
const std::string DEFAULT_INFRA2_OPTICAL_FRAME_ID  = "camera_infra2_optical_frame";
const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID   = "camera_color_optical_frame";
const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
const std::string DEFAULT_ACCEL_OPTICAL_FRAME_ID   = "camera_accel_optical_frame";
const std::string DEFAULT_GYRO_OPTICAL_FRAME_ID    = "camera_gyro_optical_frame";
const std::string DEFAULT_IMU_OPTICAL_FRAME_ID     = "camera_imu_optical_frame";

const std::string DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID = "camera_aligned_depth_to_color_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA1_FRAME_ID = "camera_aligned_depth_to_infra1_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA2_FRAME_ID = "camera_aligned_depth_to_infra2_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_FISHEYE_FRAME_ID = "camera_aligned_depth_to_fisheye_frame";

const std::string DEFAULT_UNITE_IMU_METHOD         = "";
const std::string DEFAULT_FILTERS                  = "";
const std::string DEFAULT_TOPIC_ODOM_IN            = "";

const float ROS_DEPTH_SCALE = 0.001;

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
  std::cout << "Realsense::Load\n";
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
      igndbg << "Starting BaseRealsense device\n";
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
      "Unsupported Realsense with Product ID: 0x" << pidStr << std::endl;
    break;
  }

  this->realSenseCamera->Start();
}
