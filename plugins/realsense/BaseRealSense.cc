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
#include <librealsense2/rs_advanced_mode.hpp>
#include <ignition/common/Console.hh>

#include <algorithm>
#include <cctype>

#include "BaseRealSense.hh"

using namespace ignition;
using namespace launch;

/////////////////////////////////////////////////
BaseRealSenseCamera::BaseRealSenseCamera(rs2::device _dev,
    const std::string &_serialNumber)
  : rs2Dev(_dev),
    serialNumber(_serialNumber)
{
  igndbg << "Serial number[" << this->serialNumber << "]\n";

  // Types for depth stream
  // this->imageFormat[RS2_STREAM_DEPTH] = CV_16UC1;

  // Ignition message type
  // this->encoding[RS2_STREAM_DEPTH] = ignition::msgs::TYPE_16UC1;

  this->unitStepSize[RS2_STREAM_DEPTH] = sizeof(uint16_t);

  this->streamName[RS2_STREAM_DEPTH] = "depth";
  //this->depthAlignedEncoding[RS2_STREAM_DEPTH] =
  //  sensor_msgs::imagethis->encodings::TYPE_16UC1;

  // Infrared stream
  this->format[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;

  // CVBridge type
  // this->imageFormat[RS2_STREAM_INFRARED] = CV_8UC1;

  // ROS message type
  //this->encoding[RS2_STREAM_INFRARED] =
  //  sensor_msgs::imagethis->encodings::MONO8;

  // sensor_msgs::ImagePtr row step size
  this->unitStepSize[RS2_STREAM_INFRARED] = sizeof(uint8_t);

  this->streamName[RS2_STREAM_INFRARED] = "infra";

  //this->depthAlignedEncoding[RS2_STREAM_INFRARED] =
  //  sensor_msgs::imagethis->encodings::TYPE_16UC1;

  // Types for color stream
  // this->imageFormat[RS2_STREAM_COLOR] = CV_8UC3;    // CVBridge type
  // this->encoding[RS2_STREAM_COLOR] = sensor_msgs::imagethis->encodings::RGB8; // ROS message type
  this->unitStepSize[RS2_STREAM_COLOR] = 3; // sensor_msgs::ImagePtr row step size
  this->streamName[RS2_STREAM_COLOR] = "color";
  //_depth_alignedthis->encoding[RS2_STREAM_COLOR] =
  //  sensor_msgs::imagethis->encodings::TYPE_16UC1;

  // Types for fisheye stream
  // this->imageFormat[RS2_STREAM_FISHEYE] = CV_8UC1;    // CVBridge type
  // this->encoding[RS2_STREAM_FISHEYE] = sensor_msgs::imagethis->encodings::MONO8; // ROS message type
  this->unitStepSize[RS2_STREAM_FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
  this->streamName[RS2_STREAM_FISHEYE] = "fisheye";
  // _depth_alignedthis->encoding[RS2_STREAM_FISHEYE] = sensor_msgs::imagethis->encodings::TYPE_16UC1;

  // Types for Motion-Module streams
  this->streamName[RS2_STREAM_GYRO] = "gyro";

  this->streamName[RS2_STREAM_ACCEL] = "accel";

  this->streamName[RS2_STREAM_POSE] = "pose";

  this->monitorOptions = {RS2_OPTION_ASIC_TEMPERATURE,
    RS2_OPTION_PROJECTOR_TEMPERATURE};

  this->SetParameters();
  this->SetupDevice();

  /* First pass
    this->SetupFilters();
    this->SetupErrorCallback();
    this->EnableDevices();
    this->SetupPublishers();
    this->SetupStreams();
    this->SetBaseStream();
    this->RegisterAutoExposureROIOptions(_node_handle);
    this->PublishStaticTransforms();
    this->PublishIntrinsics();
    */
}

//////////////////////////////////////////////////
BaseRealSenseCamera::~BaseRealSenseCamera()
{
  // Kill dynamic transform thread
  if (this->tfThread)
    this->tfThread->join();

  this->running = false;
  this->monitorConditionVariable.notify_one();

  if (this->monitorThread && this->monitorThread->joinable())
    this->monitorThread->join();
}

//////////////////////////////////////////////////
void BaseRealSenseCamera::Start()
{
  // This is for temperature diagnostics.
  // \todo Add the concept of diagnostics to Ignition
  /*for (rs2_option option : this->monitorOptions)
  {
    _temperature_nodes.push_back(
        {option, std::make_shared<TemperatureDiagnostics>(
            rs2_option_to_string(option), this->serialNumber)});
  }*/

  using namespace std::chrono_literals;

  // This function is used by the monitor thread to periodically send
  // temperature diagnostics.
  std::function<void()> func = [this]()
  {
    std::mutex mutex;
    std::unique_lock<std::mutex> lock(mutex);

    this->running = true;
    while (this->running)
    {
      this->monitorConditionVariable.wait_for(lock, 1s,
          [&] {return !this->running;});

      if (this->running)
        this->PublishTemperature();
    }
  };

  this->monitorThread = std::make_shared<std::thread>(func);
}

//////////////////////////////////////////////////
void BaseRealSenseCamera::PublishTemperature()
{
  /* rs2::options sensor(this->sensors[_base_stream]);
  for (OptionTemperatureDiag optionDiag : _temperature_nodes)
  {
    rs2_option option(optionDiag.first);
    if (sensor.supports(option))
    {
      try
      {
        optionDiag.second->update(sensor.get_option(option));
      }
      catch(const std::exception& e)
      {
        igndbg << "Failed checking for temperature[" << e.what() << "]\n";
      }
    }
  }*/
}

//////////////////////////////////////////////////
void BaseRealSenseCamera::SetParameters()
{
  igndbg << "Setting RealSense parameters\n";

  // Todo: Implement parameter server
  // _pnh.param("align_depth", this->alignDepth, ALIGN_DEPTH);
  // _pnh.param("enable_pointcloud", this->pointCloud, POINTCLOUD);
  // _pnh.param("pointcloud_texture_stream", pcTextureStream,
  //   std::string("RS2_STREAM_COLOR"));
  // _pnh.param("pointcloud_texture_index", pcTextureIdx, 0);
  // _pnh.param("filters", this->filtersStr, DEFAULT_FILTERS);
  // _pnh.param("publish_tf", this->publishTf, PUBLISH_TF);
  // _pnh.param("tf_publish_rate", this->tfPublishRate, TF_PUBLISH_RATE);

  // _pnh.param("enable_sync", this->syncFrames, SYNC_FRAMES);
  // _pnh.param("json_file_path", this->jsonFilePath, std::string(""));

  std::string pcTextureStream("RS2_STREAM_COLOR");

  int pcTextureIdx = 0;
  this->pointCloudTexture = StreamIndexPair{
    Rs2StringToStream(pcTextureStream), pcTextureIdx};

  this->syncFrames = (this->pointCloud || this->alignDepth ||
      this->filtersStr.size() > 0);

  /*for (auto& stream : IMAGE_STREAMS)
  {
    std::string param_name(this->streamName[stream.first] + "_width");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _pnh.param(param_name, _width[stream], IMAGE_WIDTH);
    param_name = this->streamName[stream.first] + "_height";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _pnh.param(param_name, _height[stream], IMAGE_HEIGHT);
    param_name = this->streamName[stream.first] + "_fps";
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _pnh.param(param_name, _fps[stream], IMAGE_FPS);
    param_name = "enable_" + STREAM_NAME(stream);
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _pnh.param(param_name, _enable[stream], true);
  }*/

  /*for (auto& stream : HID_STREAMS)
  {
    std::string param_name(this->streamName[stream.first] + "_fps");
    ROS_DEBUG_STREAM("reading parameter:" << param_name);
    _pnh.param(param_name, _fps[stream], IMU_FPS);
    param_name = "enable_" + STREAM_NAME(stream);
    _pnh.param(param_name, _enable[stream], ENABLE_IMU);
    ROS_DEBUG_STREAM("_enable[" << this->streamName[stream.first] << "]:" << _enable[stream]);
  }
  _pnh.param("base_frame_id", this->baseframeId, DEFAULT_BASE_FRAME_ID);
  _pnh.param("odom_frame_id", this->odomFrameId, DEFAULT_ODOM_FRAME_ID);
  */

  /*std::vector<StreamIndexPair> streams(IMAGE_STREAMS);
  streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
  for (auto& stream : streams)
  {
    std::string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_frame_id").str());
    _pnh.param(param_name, this->frameId[stream], FRAME_ID(stream));
    ROS_DEBUG_STREAM("frame_id: reading parameter:" << param_name << " : " << this->frameId[stream]);
    param_name = static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "optical_frame_id").str();
    _pnh.param(param_name, this->opticalFrameId[stream], OPTICAL_FRAME_ID(stream));
    ROS_DEBUG_STREAM("optical: reading parameter:" << param_name << " : " << this->opticalFrameId[stream]);
  }

  std::string unite_imu_method_str("");
  _pnh.param("unite_imu_method", unite_imu_method_str, DEFAULT_UNITE_IMU_METHOD);
  if (unite_imu_method_str == "linear_interpolation")
    _imu_sync_method = imu_sync_method::LINEAR_INTERPOLATION;
  else if (unite_imu_method_str == "copy")
    _imu_sync_method = imu_sync_method::COPY;
  else
    _imu_sync_method = imu_sync_method::NONE;

  if (_imu_sync_method > imu_sync_method::NONE)
  {
    _pnh.param("imu_optical_frame_id", this->opticalFrameId[GYRO], DEFAULT_IMU_OPTICAL_FRAME_ID);
  }

  for (auto& stream : IMAGE_STREAMS)
  {
    if (stream == DEPTH) continue;
    if (stream.second > 1) continue;
    std::string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << "aligned_depth_to_" << STREAM_NAME(stream) << "_frame_id").str());
    _pnh.param(param_name, this->depthAlignedFrameId[stream], ALIGNED_DEPTH_TO_FRAME_ID(stream));
  }

  _pnh.param("allow_no_texture_points", _allow_no_texture_points, ALLOW_NO_TEXTURE_POINTS);
  _pnh.param("clip_distance", _clipping_distance, static_cast<float>(-1.0));
  _pnh.param("linear_accel_cov", _linear_accel_cov, static_cast<double>(0.01));
  _pnh.param("angular_velocity_cov", _angular_velocity_cov, static_cast<double>(0.01));
  _pnh.param("hold_back_imu_for_frames", _hold_back_imu_for_frames, HOLD_BACK_IMU_FOR_FRAMES);
  _pnh.param("publish_odom_tf", _publish_odom_tf, PUBLISH_ODOM_TF);
  */
}

//////////////////////////////////////////////////
/*void BaseRealSenseCamera::SetupDevice()
{
  try
  {
    if (!this->jsonFilePath.empty())

      for (auto it = frameset.begin(); it != frameset.end(); ++it)
      {
        auto f = (*it);
        auto stream_type = f.get_profile().stream_type();
        auto streamIndex = f.get_profile().streamIndex();
        auto stream_format = f.get_profile().format();
        auto stream_unique_id = f.get_profile().unique_id();

        //ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu", rs2_stream_to_string(stream_type), streamIndex, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frameTime, t.toNSec());
        this->RunFirstFrameInitialization(stream_type);
      }

      // Clip depth_frame for max range:
      rs2::depth_frame depthFrame = frameset.get_depth_frame();
      if (depthFrame && this->clippingDistance > 0)
      {
        clip_depth(depth_frame, this->clippingDistance);
      }

      igndbg << "num_filters: " << static_cast<int>(_filters.size())
        << std::endl;

      for (std::vector<NamedFilter>::const_iterator filterIt =
          this->filters.begin(); filterIt != this->filters.end(); filterIt++)
      {
        igndbg << "Applying filter: "  << filterIt->_name.c_str() << std::endl;
        frameset = filterIt->_filter->process(frameset);
      }

      igndbg << "List of frameset after applying filters: size: "
        << static_cast<int>(frameset.size()) << std::endl;

      for (auto it = frameset.begin(); it != frameset.end(); ++it)
      {
        auto f = (*it);
        auto stream_type = f.get_profile().stream_type();
        auto streamIndex = f.get_profile().streamIndex();
        auto stream_format = f.get_profile().format();
        auto stream_unique_id = f.get_profile().unique_id();

        //ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu", rs2_stream_to_string(stream_type), streamIndex, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frameTime, t.toNSec());
      }

      igndbg << "END OF LIST\n";
      // TODO - Fix the following issue:
      // Currently publishers are set using a map of stream type and index only.
      // It means that colorized depth image <DEPTH, 0, Z16> and
      // colorized depth image <DEPTH, 0, RGB>
      // use the same publisher.
      // As a workaround we remove the earlier one, the original one,
      // assuming that if colorizer filter is
      // set it means that that's what the client wants.
      bool pointsInSet(false);
      std::vector<rs2::frame> framesToPublish;
      std::vector<StreamIndexPair> is_in_set;

      for (auto it = frameset.begin(); it != frameset.end(); ++it)
      {
        auto f = (*it);
        auto streamType = f.get_profile().streamType();
        auto streamIndex = f.get_profile().streamIndex();
        auto stream_format = f.get_profile().format();

        if (f.is<rs2::points>())
        {
          if (!pointsInSet)
          {
            pointsInSet = true;
            framesToPublish.push_back(f);
          }
          continue;
        }

        StreamIndexPair sip{streamType,streamIndex};
        if (std::find(is_in_set.begin(), is_in_set.end(), sip) == is_in_set.end())
        {
          is_in_set.push_back(sip);
          framesToPublish.push_back(f);
        }

        if (this->alignDepth && streamType == RS2_STREAM_DEPTH &&
            stream_format == RS2_FORMAT_Z16)
        {
          is_depth_arrived = true;
        }
      }

      for (auto it = framesToPublish.begin(); it != framesToPublish.end(); ++it)
      {
        auto f = (*it);
        auto streamType = f.get_profile().streamType();
        auto streamIndex = f.get_profile().streamIndex();
        auto stream_format = f.get_profile().format();

        ROS_DEBUG("Frameset contain (%s, %d, %s) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
            rs2_stream_to_string(streamType), streamIndex, rs2_format_to_string(stream_format), frame.get_frame_number(), frame_time, t.toNSec());

        if (f.is<rs2::points>())
        {
          if (0 != _pointcloud_publisher.getNumSubscribers())
          {
            ROS_DEBUG("Publish pointscloud");
            this->PublishPointCloud(f.as<rs2::points>(), t, frameset);
          }
          continue;
        }
        StreamIndexPair sip{stream_type,streamIndex};
        // this->PublishFrame(f, t,
        //                 sip,
        //                 _image,
        //                 this->infoPublisher,
        //                 _image_publishers, _seq,
        //                 _camera_info, this->opticalFrameId,
        //                 this->encoding);
      }

      if (this->alignDepth && is_depth_arrived)
      {
        ROS_DEBUG("publishAlignedDepthToOthers(...)");
        this->PublishAlignedDepthToOthers(frameset, t);
      }
    }
    else if (frame.is<rs2::video_frame>())
    {
      auto stream_type = frame.get_profile().stream_type();
      auto streamIndex = frame.get_profile().streamIndex();
      ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
          rs2_stream_to_string(stream_type), streamIndex, frame.get_frame_number(), frame_time, t.toNSec());
      this->RunFirstFrameInitialization(stream_type);

      StreamIndexPair sip{stream_type,streamIndex};
      if (frame.is<rs2::depth_frame>())
      {
        if (_clipping_distance > 0)
        {
          clip_depth(frame, _clipping_distance);
        }
      }
      // this->PublishFrame(frame, t,
      //                 sip,
      //                 _image,
      //                 this->infoPublisher,
      //                 _image_publishers, _seq,
      //                 _camera_info, this->opticalFrameId,
      //                 this->encoding);
    }
  }
  catch(const std::exception& ex)
  {
    ignerr << "An error has occurred during frame callback: " << ex.what();
  }

  this->yncedImuPublisher->Resume();
};
*/







/////////////////////////////////////////////////
void BaseRealSenseCamera::ToggleSensors(bool _enabled)
{
  for (auto it = this->sensors.begin(); it != this->sensors.end(); ++it)
  {
    auto &sens = this->sensors[it->first];
    try
    {
      if (_enabled)
        sens.start(this->pipelineSyncer);
      else
        sens.stop();
    }
    catch(const rs2::wrong_api_call_sequence_error& ex)
    {
      igndbg << "toggleSensors[" << ex.what() << "]\n";
    }
  }
}

//////////////////////////////////////////////////
void BaseRealSenseCamera::SetupErrorCallback()
{
  for (auto &&s : this->rs2Dev.query_sensors())
  {
    s.set_notifications_callback([&](const rs2::notification &_notification)
    {
      std::vector<std::string> errorStrings(
        {"RT IC2 Config error",
           "Left IC2 Config error"
        });

      if (_notification.get_severity() >= RS2_LOG_SEVERITY_ERROR)
      {
        ignwarn << "Hardware Notification:"
          << _notification.get_description()
          << "," << _notification.get_timestamp()
          << "," <<  _notification.get_severity()
          << "," << _notification.get_category()
          << std::endl;
      }

      if (errorStrings.end() !=
          std::find_if(errorStrings.begin(), errorStrings.end(),
            [&_notification] (std::string err) {
              return (_notification.get_description().find(err) !=
                      std::string::npos);
            }))
      {
        ignerr << "Performing Hardware Reset.\n";
        this->rs2Dev.hardware_reset();
      }
    });
  }
}


//////////////////////////////////////////////////
rs2_stream BaseRealSenseCamera::Rs2StringToStream(const std::string &_str) const
{
  if (_str == "RS2_STREAM_ANY")
    return RS2_STREAM_ANY;
  else if (_str == "RS2_STREAM_COLOR")
    return RS2_STREAM_COLOR;
  else if (_str == "RS2_STREAM_INFRARED")
    return RS2_STREAM_INFRARED;
  else if (_str == "RS2_STREAM_FISHEYE")
    return RS2_STREAM_FISHEYE;

  ignerr << "Invalid string[" << _str << "]\n";
  return RS2_STREAM_ANY;
}










// GARBAGE
//
/*
namespace realsense2_camera
{
  template <typename K, typename V>
    std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m)
    {
      os << '{';
      for (const auto& kv : m)
      {
        os << " {" << kv.first << ": " << kv.second << '}';
      }
      os << " }";
      return os;
    }
}*/

// StreamIndexPair sip{stream_type, streamIndex};
#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << this->streamName[sip.first] << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()
#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

//////////////////////////////////////////////////
bool kIsCheckbox(rs2::options _sensor, rs2_option _option)
{
  rs2::option_range opRange = _sensor.get_option_range(_option);
  return opRange.max == 1.0f &&
         opRange.min == 0.0f &&
         opRange.step == 1.0f;
}

//////////////////////////////////////////////////
bool kIsEnumOption(rs2::options _sensor, rs2_option _option)
{
  rs2::option_range opRange = _sensor.get_option_range(_option);

  if (opRange.step < 0.001f)
    return false;

  for (auto i = opRange.min; i <= opRange.max; i += opRange.step)
  {
    if (_sensor.get_option_value_description(_option, i) == nullptr)
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool kIsIntOption(rs2::options _sensor, rs2_option _option)
{
  rs2::option_range opRange = _sensor.get_option_range(_option);
  return (opRange.step == 1.0);
}

//////////////////////////////////////////////////
std::map<std::string, int> kGetEnumMethod(rs2::options _sensor,
    rs2_option _option)
{
  std::map<std::string, int> dict; // An enum to set size
  if (kIsEnumOption(_sensor, _option))
  {
    rs2::option_range opRange = _sensor.get_option_range(_option);
    const auto opRangeMin = int(opRange.min);
    const auto opRangeMax = int(opRange.max);
    const auto opRangeStep = int(opRange.step);
    for (auto val = opRangeMin; val <= opRangeMax; val += opRangeStep)
    {
      dict[_sensor.get_option_value_description(_option, val)] = val;
    }
  }

  return dict;
}


/////////////////////////////////////////////////
/*SyncedImuPublisher::SyncedImuPublisher(
    ignition::Transport::Node::Publisher *_imuPublisher,
    std::size_t _waitingListSize)
  : _publisher(_imuPublisher),
    _waitingListSize(_waitingListSize)
{
}

/////////////////////////////////////////////////
SyncedImuPublisher::~SyncedImuPublisher()
{
  this->PublishPendingMessages();
}

/////////////////////////////////////////////////
bool SyncedImuPublisher::Publish(const ignition::msgs::IMU &_msg)
{
  std::lock_guard<std::mutex> lock_guard(this->mutex);
  if (this->pauseMode)
  {
    if (this->pendingMessages.size() >= this->waitingListSize)
    {
      ignerr << "SyncedImuPublisher inner list reached maximum size of "
       << this->pendingMessages.size() << std::endl;
      return false;
    }
    this->pendingMessages.push(_msg);
  }
  else
  {
    this->publisher.Publish(_msg);
  }

  return true;
}

/////////////////////////////////////////////////
void SyncedImuPublisher::Pause()
{
  if (!this->enabled)
    return;

  std::lock_guard<std::mutex> lock_guard(_mutex);
  this->pauseMode = true;
}

/////////////////////////////////////////////////
void SyncedImuPublisher::Resume()
{
  std::lock_guard<std::mutex> lock_guard(_mutex);
  this->PublishPendingMessages();
  this->pauseMode = false;
}

/////////////////////////////////////////////////
void SyncedImuPublisher::PublishPendingMessages()
{
  // igninfo << "publish imu: " << this->pendingMessages.size() << std::endl;
  while (!this->pendingMessages.empty())
  {
    this->publisher.Publish(this->pendingMessages.front());
    // igninfo << "iid2:" << imu_msg.header.seq << ", time: "
    // << std::setprecision (20) << imu_msg.header.stamp.toSec());
    this->pendingMessages.pop();
  }
}
*/



//////////////////////////////////////////////////
/*void BaseRealSenseNode::RunFirstFrameInitialization(rs2_stream stream_type)
{
    if (_is_first_frame[stream_type])
    {
        ROS_DEBUG_STREAM("RunFirstFrameInitialization: " << _video_functions_stack.size() << ", " << rs2_stream_to_string(stream_type));
        _is_first_frame[stream_type] = false;
        if (!_video_functions_stack[stream_type].empty())
        {
            std::thread t = std::thread([=]()
            {
                while (!_video_functions_stack[stream_type].empty())
                {
                    _video_functions_stack[stream_type].back()();
                    _video_functions_stack[stream_type].pop_back();
                }
            });
            t.detach();
        }
    }
}*/

//////////////////////////////////////////////////
//Same as ros::names::isValidCharInName, but re-implemented here because it's not exposed.
/*bool isValidCharInName(char c)
{
    return std::isalnum(c) || c == '/' || c == '_';
}*/

//////////////////////////////////////////////////
// ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names), so we replace them here with underscores.
/*std::string createGraphResourceName(const std::string &original_name)
{
  std::string fixed_name = original_name;
  std::transform(fixed_name.begin(), fixed_name.end(), fixed_name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  std::replace_if(fixed_name.begin(), fixed_name.end(), [](const char c) { return !isValidCharInName(c); },
                  '_');
  return fixed_name;
}*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::SetAutoExposureRoi(const std::string option_name, rs2::sensor sensor, int new_value)
{
  rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
  if (option_name == "left")
    auto_exposure_roi.min_x = new_value;
  else if (option_name == "right")
    auto_exposure_roi.max_x = new_value;
  else if (option_name == "top")
    auto_exposure_roi.min_y = new_value;
  else if (option_name == "bottom")
    auto_exposure_roi.max_y = new_value;
  else
  {
    ROS_WARN_STREAM("Invalid option_name: " << option_name << " while setting auto exposure ROI.");
    return;
  }
  this->SetSensorAutoExposureRoi(sensor);
}*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::SetSensorAutoExposureRoi(rs2::sensor sensor)
{
    const rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
    try
    {
        sensor.as<rs2::roi_sensor>().set_region_of_interest(auto_exposure_roi);
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::ReadAndSetDynamicParam(
    ros::NodeHandle& nh1,
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec,
    const std::string option_name, const int min_val,
    const int max_val, rs2::sensor sensor,
    int* option_value)
{
  nh1.param(option_name, *option_value, *option_value); //param (const std::string &param_name, T &param_val, const T &default_val) const
  if (*option_value < min_val) *option_value = min_val;
  if (*option_value > max_val) *option_value = max_val;

  ddynrec->registerVariable<int>(
      option_name, *option_value, [this, sensor, option_name](int new_value){this->SetAutoExposureRoi(option_name, sensor, new_value);},
      "auto-exposure " + option_name + " coordinate", min_val, max_val);
}*/

/*void BaseRealSenseNode::RegisterAutoExposureROIOptions(ros::NodeHandle& nh)
{
  for (const std::pair<StreamIndexPair, std::vector<rs2::stream_profile>>& profile : this->enabledProfiles)
  {
    rs2::sensor sensor = this->sensors[profile.first];
    std::string module_base_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
    if (sensor.is<rs2::roi_sensor>() && _auto_exposure_roi.find(module_base_name) == _auto_exposure_roi.end())
    {
      int max_x(_width[profile.first]-1);
      int max_y(_height[profile.first]-1);

      std::string module_name = createGraphResourceName(module_base_name) +"/auto_exposure_roi";
      ros::NodeHandle nh1(nh, module_name);
      std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);

      _auto_exposure_roi[module_base_name] = {0, 0, max_x, max_y};
      rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[module_base_name]);
      this->ReadAndSetDynamicParam(nh1, ddynrec, "left", 0, max_x, sensor, &(auto_exposure_roi.min_x));
      this->ReadAndSetDynamicParam(nh1, ddynrec, "right", 0, max_x, sensor, &(auto_exposure_roi.max_x));
      this->ReadAndSetDynamicParam(nh1, ddynrec, "top", 0, max_y, sensor, &(auto_exposure_roi.min_y));
      this->ReadAndSetDynamicParam(nh1, ddynrec, "bottom", 0, max_y, sensor, &(auto_exposure_roi.max_y));

      ddynrec->publishServicesTopics();
      _ddynrec.push_back(ddynrec);

      // Initiate the call to this->SetSensorAutoExposureRoi, after the first frame arrive.
      rs2_stream stream_type = profile.first.first;
      _video_functions_stack[stream_type].push_back([this, sensor](){this->SetSensorAutoExposureRoi(sensor);});
      _is_first_frame[stream_type] = true;
    }
  }
}*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::RegisterDynamicOption(
    ros::NodeHandle& nh, rs2::options sensor, std::string& module_name)
{
  ros::NodeHandle nh1(nh, module_name);
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);
  for (auto i = 0; i < RS2_OPTION_COUNT; i++)
  {
    rs2_option option = static_cast<rs2_option>(i);
    const std::string option_name(createGraphResourceName(rs2_option_to_string(option)));
    if (!sensor.supports(option) || sensor.is_option_read_only(option))
    {
      continue;
    }
    if (kIsCheckbox(sensor, option))
    {
      auto option_value = bool(sensor.get_option(option));
      if (nh1.param(option_name, option_value, option_value))
      {
        sensor.set_option(option, option_value);
      }
      ddynrec->registerVariable<bool>(
          option_name, option_value,
          [option, sensor](bool new_value) { sensor.set_option(option, new_value); },
          sensor.get_option_description(option));
      continue;
    }
    const auto enum_dict = kGetEnumMethod(sensor, option);
    if (enum_dict.empty())
    {
      rs2::option_range op_range = sensor.get_option_range(option);
      const auto sensor_option_value = sensor.get_option(option);
      auto option_value = sensor_option_value;
      if (nh1.param(option_name, option_value, option_value))
      {
        if (option_value < op_range.min || op_range.max < option_value)
        {
          ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
              << " outside the range [" << op_range.min << ", " << op_range.max
              << "]. Using current sensor value " << sensor_option_value << " instead.");
          option_value = sensor_option_value;
        }
        else
        {
          sensor.set_option(option, option_value);
        }
      }
      if (kIsIntOption(sensor, option))
      {
        ddynrec->registerVariable<int>(
            option_name, int(option_value),
            [option, sensor](int new_value) { sensor.set_option(option, new_value); },
            sensor.get_option_description(option), int(op_range.min), int(op_range.max));
      }
      else
      {
        if (i == RS2_OPTION_DEPTH_UNITS)
        {
          if (ROS_DEPTH_SCALE >= op_range.min && ROS_DEPTH_SCALE <= op_range.max)
          {
            sensor.set_option(option, ROS_DEPTH_SCALE);
            op_range.min = ROS_DEPTH_SCALE;
            op_range.max = ROS_DEPTH_SCALE;

            _depth_scale_meters = ROS_DEPTH_SCALE;
          }
        }
        else
        {
          ddynrec->registerVariable<double>(
              option_name, option_value,
              [option, sensor](double new_value) { sensor.set_option(option, new_value); },
              sensor.get_option_description(option), double(op_range.min), double(op_range.max));
        }
      }
    }
    else
    {
      const auto sensor_option_value = sensor.get_option(option);
      auto option_value = int(sensor_option_value);
      if (nh1.param(option_name, option_value, option_value))
      {
        if (std::find_if(enum_dict.cbegin(), enum_dict.cend(),
              [&option_value](const std::pair<std::string, int>& kv) {
              return kv.second == option_value;
              }) == enum_dict.cend())
        {
          ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
              << " that is not in the enum " << enum_dict
              << ". Using current sensor value " << sensor_option_value << " instead.");
          option_value = sensor_option_value;
        }
        else
        {
          sensor.set_option(option, option_value);
        }
      }
      ddynrec->registerEnumVariable<int>(
          option_name, option_value,
          [option, sensor](int new_value) { sensor.set_option(option, new_value); },
          sensor.get_option_description(option), enum_dict);
    }
  }
  ddynrec->publishServicesTopics();
  _ddynrec.push_back(ddynrec);
}*/



/////////////////////////////////////////////////
/*void BaseRealSenseNode::SetupPublishers()
{
  image_transport::ImageTransport image_transport(_node_handle);

  for (auto& stream : IMAGE_STREAMS)
  {
    if (_enable[stream])
    {
      std::stringstream image_raw, camera_info;
      bool rectified_image = false;
      if (stream == DEPTH || stream == INFRA1 || stream == INFRA2)
        rectified_image = true;

      std::string stream_name(STREAM_NAME(stream));
      image_raw << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
      camera_info << stream_name << "/camera_info";

      std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], stream_name, this->serialNumber));
      _image_publishers[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
      this->infoPublisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

      if (this->alignDepth && (stream != DEPTH) && stream.second < 2)
      {
        std::stringstream aligned_image_raw, aligned_camera_info;
        aligned_image_raw << "aligned_depth_to_" << stream_name << "/image_raw";
        aligned_camera_info << "aligned_depth_to_" << stream_name << "/camera_info";

        std::string alignedthis->streamName = "aligned_depth_to_" + stream_name;
        std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], alignedthis->streamName, this->serialNumber));
        _depth_aligned_image_publishers[stream] = {image_transport.advertise(aligned_image_raw.str(), 1), frequency_diagnostics};
        this->depthAlignedInfoPublisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(aligned_camera_info.str(), 1);
      }

      if (stream == DEPTH && this->pointCloud)
      {
        _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
      }
    }
  }

  this->syncedImuPublisher = std::make_shared<SyncedImuPublisher>();
  if (_imu_sync_method > imu_sync_method::NONE && _enable[GYRO] && _enable[ACCEL])
  {
    ROS_INFO("Start publisher IMU");
    this->syncedImuPublisher = std::make_shared<SyncedImuPublisher>(_node_handle.advertise<sensor_msgs::Imu>("imu", 5));
    this->syncedImuPublisher->Enable(_hold_back_imu_for_frames);
  }
  else
  {
    if (_enable[GYRO])
    {
      _imu_publishers[GYRO] = _node_handle.advertise<sensor_msgs::Imu>("gyro/sample", 100);
    }

    if (_enable[ACCEL])
    {
      _imu_publishers[ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("accel/sample", 100);
    }
  }
  if (_enable[POSE])
  {
    _imu_publishers[POSE] = _node_handle.advertise<nav_msgs::Odometry>("odom/sample", 100);
  }


  if (_enable[FISHEYE] &&
      _enable[DEPTH])
  {
    _depth_to_other_extrinsics_publishers[FISHEYE] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_fisheye", 1, true);
  }

  if (_enable[COLOR] &&
      _enable[DEPTH])
  {
    _depth_to_other_extrinsics_publishers[COLOR] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_color", 1, true);
  }

  if (_enable[INFRA1] &&
      _enable[DEPTH])
  {
    _depth_to_other_extrinsics_publishers[INFRA1] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra1", 1, true);
  }

  if (_enable[INFRA2] &&
      _enable[DEPTH])
  {
    _depth_to_other_extrinsics_publishers[INFRA2] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra2", 1, true);
  }
}
*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishAlignedDepthToOthers(
    rs2::frameset frames, const const std::chrono::time_point &_time)
{
  for (auto it = frames.begin(); it != frames.end(); ++it)
  {
    auto frame = (*it);
    auto stream_type = frame.get_profile().stream_type();

    if (RS2_STREAM_DEPTH == stream_type)
      continue;

    auto streamIndex = frame.get_profile().streamIndex();
    if (streamIndex > 1)
    {
      continue;
    }
    StreamIndexPair sip{stream_type, streamIndex};
    auto& info_publisher = this0>depthAlignedInfoPublisher.at(sip);
    auto& image_publisher = _depth_aligned_image_publishers.at(sip);

    if(0 != info_publisher.getNumSubscribers() ||
        0 != image_publisher.first.getNumSubscribers())
    {
      std::shared_ptr<rs2::align> align;
      try{
        align = _align.at(stream_type);
      }
      catch(const std::out_of_range& e)
      {
        ROS_DEBUG_STREAM("Allocate align filter for:" << rs2_stream_to_string(sip.first) << sip.second);
        align = (_align[stream_type] = std::make_shared<rs2::align>(stream_type));
      }
      rs2::frameset processed = frames.apply_filter(*align);
      rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

      publishFrame(aligned_depth_frame, t, sip,
          _depth_aligned_image,
          this->depthAlignedInfoPublisher,
          _depth_aligned_image_publishers, _depth_aligned_seq,
          _depth_aligned_camera_info, this->opticalFrameId,
          this->depthAlignedEncoding);
    }
  }
}
*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::EnableDecives()
{
  for (auto& elem : IMAGE_STREAMS)
  {
    if (_enable[elem])
    {
      auto& sens = this->sensors[elem];
      auto profiles = sens.get_stream_profiles();
      for (auto& profile : profiles)
      {
        auto video_profile = profile.as<rs2::video_stream_profile>();
        ROS_DEBUG_STREAM("Sensor profile: " <<
            "stream_type: " << rs2_stream_to_string(elem.first) << "(" << elem.second << ")" <<
            "Format: " << video_profile.format() <<
            ", Width: " << video_profile.width() <<
            ", Height: " << video_profile.height() <<
            ", FPS: " << video_profile.fps());

        if ((video_profile.stream_type() == elem.first) &&
            (_width[elem] == 0 || video_profile.width() == _width[elem]) &&
            (_height[elem] == 0 || video_profile.height() == _height[elem]) &&
            (_fps[elem] == 0 || video_profile.fps() == _fps[elem]) &&
            (_format.find(elem.first) == _format.end() || video_profile.format() == _format[elem.first] ) &&
            video_profile.streamIndex() == elem.second)
        {
          _width[elem] = video_profile.width();
          _height[elem] = video_profile.height();
          _fps[elem] = video_profile.fps();

          this->enabledProfiles[elem].push_back(profile);

          _image[elem] = cv::Mat(_height[elem], _width[elem], this->imageFormat[elem.first], cv::Scalar(0, 0, 0));

          ROS_INFO_STREAM(STREAM_NAME(elem) << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem] << ", " << "Format: " << video_profile.format());
          break;
        }
      }
      if (this->enabledProfiles.find(elem) == this->enabledProfiles.end())
      {
        ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
            " Stream: " << rs2_stream_to_string(elem.first) <<
            ", Stream Index: " << elem.second <<
            ", Width: " << _width[elem] <<
            ", Height: " << _height[elem] <<
            ", FPS: " << _fps[elem]);
        _enable[elem] = false;
      }
    }
  }
  if (this->alignDepth)
  {
    for (auto& profiles : this->enabledProfiles)
    {
      _depth_aligned_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], this->imageFormat[DEPTH.first], cv::Scalar(0, 0, 0));
      _depth_scaled_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], this->imageFormat[DEPTH.first], cv::Scalar(0, 0, 0));
    }
  }

  // Streaming HID
  for (auto& elem : HID_STREAMS)
  {
    if (_enable[elem])
    {
      auto& sens = this->sensors[elem];
      auto profiles = sens.get_stream_profiles();
      ROS_DEBUG_STREAM("Available profiles:");
      for (rs2::stream_profile& profile : profiles)
      {
        ROS_DEBUG_STREAM("type:" << rs2_stream_to_string(profile.stream_type()) <<
            " fps: " << profile.fps() << ". format: " << profile.format());
      }
      for (rs2::stream_profile& profile : profiles)
      {
        if (profile.stream_type() == elem.first &&
            (_fps[elem] == 0 || profile.fps() == _fps[elem]))
        {
          _fps[elem] = profile.fps();
          this->enabledProfiles[elem].push_back(profile);
          break;
        }
      }
      if (this->enabledProfiles.find(elem) == this->enabledProfiles.end())
      {
        std::string stream_name(STREAM_NAME(elem));
        ROS_WARN_STREAM("No mathcing profile found for " << stream_name << " with fps=" << _fps[elem]);
        ROS_WARN_STREAM("profiles found for " <<stream_name << ":");
        for (rs2::stream_profile& profile : profiles)
        {
          if (profile.stream_type() != elem.first) continue;
          ROS_WARN_STREAM("fps: " << profile.fps() << ". format: " << profile.format());
        }
        _enable[elem] = false;
      }
    }
  }
}*/

//////////////////////////////////////////////////
void BaseRealSenseCamera::SetupFilters()
{
  /*
    std::vector<std::string> filters_str;
    boost::split(filters_str, this->filtersStr, [](char c){return c == ',';});
    bool use_disparity_filter(false);
    bool use_colorizer_filter(false);
    bool use_decimation_filter(false);
    for (std::vector<std::string>::const_iterator s_iter=filters_str.begin(); s_iter!=filters_str.end(); s_iter++)
    {
        if ((*s_iter) == "colorizer")
        {
            use_colorizer_filter = true;
        }
        else if ((*s_iter) == "disparity")
        {
            use_disparity_filter = true;
        }
        else if ((*s_iter) == "spatial")
        {
            ROS_INFO("Add Filter: spatial");
            _filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>()));
        }
        else if ((*s_iter) == "temporal")
        {
            ROS_INFO("Add Filter: temporal");
            _filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>()));
        }
        else if ((*s_iter) == "hole_filling")
        {
            ROS_INFO("Add Filter: hole_filling");
            _filters.push_back(NamedFilter("hole_filling", std::make_shared<rs2::hole_filling_filter>()));
        }
        else if ((*s_iter) == "decimation")
        {
            use_decimation_filter = true;
        }
        else if ((*s_iter) == "pointcloud")
        {
            assert(this->pointCloud); // For now, it is set in getParameters()..
        }
        else if ((*s_iter).size() > 0)
        {
            ROS_ERROR_STREAM("Unknown Filter: " << (*s_iter));
            throw;
        }
    }
    if (use_disparity_filter)
    {
        ROS_INFO("Add Filter: disparity");
        _filters.insert(_filters.begin(), NamedFilter("disparity_start", std::make_shared<rs2::disparity_transform>()));
        _filters.push_back(NamedFilter("disparity_end", std::make_shared<rs2::disparity_transform>(false)));
        ROS_INFO("Done Add Filter: disparity");
    }
    if (use_decimation_filter)
    {
      ROS_INFO("Add Filter: decimation");
      _filters.insert(_filters.begin(),NamedFilter("decimation", std::make_shared<rs2::decimation_filter>()));
    }
    if (use_colorizer_filter)
    {
        ROS_INFO("Add Filter: colorizer");
        _filters.push_back(NamedFilter("colorizer", std::make_shared<rs2::colorizer>()));

        // Types for depth stream
        this->imageFormat[DEPTH.first] = this->imageFormat[COLOR.first];    // CVBridge type
        this->encoding[DEPTH.first] = this->encoding[COLOR.first]; // ROS message type
        this->unitStepSize[DEPTH.first] = this->unitStepSize[COLOR.first]; // sensor_msgs::ImagePtr row step size

        _width[DEPTH] = _width[COLOR];
        _height[DEPTH] = _height[COLOR];
        _image[DEPTH] = cv::Mat(_height[DEPTH], _width[DEPTH], this->imageFormat[DEPTH.first], cv::Scalar(0, 0, 0));
    }
    if (this->pointCloud)
    {
      ROS_INFO("Add Filter: pointcloud");
        _filters.push_back(NamedFilter("pointcloud", std::make_shared<rs2::pointcloud>(this->pointCloudTexture.first, this->pointCloudTexture.second)));
    }
    ROS_INFO("num_filters: %d", static_cast<int>(_filters.size()));
    */
}


/*
cv::Mat& BaseRealSenseNode::fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image)
{
    static const float meter_to_mm = 0.001f;
    if (fabs(_depth_scale_meters - meter_to_mm) < 1e-6)
    {
        to_image = from_image;
        return to_image;
    }

    if (to_image.size() != from_image.size())
    {
        to_image.create(from_image.rows, from_image.cols, from_image.type());
    }

    CV_Assert(from_image.depth() == this->imageFormat[RS2_STREAM_DEPTH]);

    int nRows = from_image.rows;
    int nCols = from_image.cols;

    if (from_image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    const uint16_t* p_from;
    uint16_t* p_to;
    for( i = 0; i < nRows; ++i)
    {
        p_from = from_image.ptr<uint16_t>(i);
        p_to = to_image.ptr<uint16_t>(i);
        for ( j = 0; j < nCols; ++j)
        {
            p_to[j] = p_from[j] * _depth_scale_meters / meter_to_mm;
        }
    }
    return to_image;
}
*/

/*
void BaseRealSenseNode::clip_depth(rs2::depth_frame depth_frame, float clipping_dist)
{
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    uint16_t clipping_value = static_cast<uint16_t>(clipping_dist / _depth_scale_meters);

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    #endif
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Check if the depth value is greater than the threashold
            if (p_depth_frame[depth_pixel_index] > clipping_value)
            {
                p_depth_frame[depth_pixel_index] = 0; //Set to invalid (<=0) value.
            }
        }
    }
}
*/

/*
sensor_msgs::Imu BaseRealSenseNode::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sensor_msgs::Imu imu_msg;
    ros::Time t(gyro_data.m_time);
    imu_msg.header.seq = 0;
    imu_msg.header.stamp = t;

    imu_msg.angular_velocity.x = gyro_data.m_data.x();
    imu_msg.angular_velocity.y = gyro_data.m_data.y();
    imu_msg.angular_velocity.z = gyro_data.m_data.z();

    imu_msg.linear_acceleration.x = accel_data.m_data.x();
    imu_msg.linear_acceleration.y = accel_data.m_data.y();
    imu_msg.linear_acceleration.z = accel_data.m_data.z();
    return imu_msg;
}*/

/*
template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}
*/

/*
//////////////////////////////////////////////////
void BaseRealSenseNode::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs)
{
    static std::deque<CimuData> _imu_history;
    _imu_history.push_back(imu_data);
    StreamIndexPair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;

    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size())
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL)
        {
            accel0 = crnt_imu;
        }
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL)
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time - accel0.m_time;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time - accel0.m_time) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        }
        else if (accel0.is_set() && crnt_imu.m_time >= accel0.m_time && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}
*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs)
{
    StreamIndexPair type(imu_data.m_type);

    static CimuData _accel_data(ACCEL, {0,0,0}, -1.0);
    if (ACCEL == type)
    {
        _accel_data = imu_data;
        return;
    }
    if (_accel_data.m_time < 0)
        return;

    imu_msgs.push_back(CreateUnitedMessage(_accel_data, imu_data));
}*/

/*
//////////////////////////////////////////////////
void BaseRealSenseNode::ImuMessage_AddDefaultValues(sensor_msgs::Imu& imu_msg)
{
    imu_msg.header.frame_id = this->opticalFrameId[GYRO];
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
    imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};
}
*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::ImuCallbackSnc(
    rs2::frame frame, imu_sync_method sync_method)
{
    static std::mutex m_mutex;
    static int seq = 0;

    m_mutex.lock();

    auto stream = frame.get_profile().stream_type();
    auto streamIndex = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();

    bool placeholder_false(false);
    if (this->isInitializedTimeBase.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    seq += 1;
    double elapsed_camera_ms = (
      //ms
      frame_time -
      //ms
      _camera_time_base) / 1000.0;

    if (0 != this->syncedImuPublisher->getNumSubscribers())
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(streamIndex, v, elapsed_camera_ms);
        std::deque<sensor_msgs::Imu> imu_msgs;
        switch (sync_method)
        {
            case NONE: //Cannot really be NONE. Just to avoid compilation warning.
            case COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::Imu imu_msg = imu_msgs.front();
            ros::Time t(this->timeBase.toSec() + imu_msg.header.stamp.toSec());
            imu_msg.header.seq = seq;
            imu_msg.header.stamp = t;
            ImuMessage_AddDefaultValues(imu_msg);
            this->syncedImuPublisher->Publish(imu_msg);
            ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
            imu_msgs.pop_front();
        }
    }
    m_mutex.unlock();
};*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::ImuCallback(rs2::frame frame)
{
    auto stream = frame.get_profile().stream_type();
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (this->isInitializedTimeBase.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().streamIndex(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    auto streamIndex = (stream == GYRO.first)?GYRO:ACCEL;
    if (0 != _imu_publishers[streamIndex].getNumSubscribers())
    {
        double elapsed_camera_ms = (
          //ms
          frame_time -
          //ms
          _camera_time_base) / 1000.0;
        ros::Time t(this->timeBase.toSec() + elapsed_camera_ms);

        auto imu_msg = sensor_msgs::Imu();
        ImuMessage_AddDefaultValues(imu_msg);
        imu_msg.header.frame_id = this->opticalFrameId[streamIndex];

        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        if (GYRO == streamIndex)
        {
            imu_msg.angular_velocity.x = crnt_reading.x;
            imu_msg.angular_velocity.y = crnt_reading.y;
            imu_msg.angular_velocity.z = crnt_reading.z;
        }
        else if (ACCEL == streamIndex)
        {
            imu_msg.linear_acceleration.x = crnt_reading.x;
            imu_msg.linear_acceleration.y = crnt_reading.y;
            imu_msg.linear_acceleration.z = crnt_reading.z;
        }
        _seq[streamIndex] += 1;
        imu_msg.header.seq = _seq[streamIndex];
        imu_msg.header.stamp = t;
        _imu_publishers[streamIndex].publish(imu_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PoseCallback(rs2::frame frame)
{
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (this->isInitializedTimeBase.compare_exchange_strong(placeholder_false, true) )
    {
        setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().streamIndex(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
    const auto& streamIndex(POSE);
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    double elapsed_camera_ms = (frame_time - _camera_time_base) / 1000.0;
    ros::Time t(this->timeBase.toSec() + elapsed_camera_ms);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = this->odomFrameId;
    msg.child_frame_id = this->frameId[POSE];
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;

    if (_publish_odom_tf) br.sendTransform(msg);

    if (0 != _imu_publishers[streamIndex].getNumSubscribers())
    {
        double cov_pose(_linear_accel_cov * pow(10, 3-(int)pose.tracker_confidence));
        double cov_twist(_angular_velocity_cov * pow(10, 1-(int)pose.tracker_confidence));

        geometry_msgs::Vector3Stamped v_msg;
        v_msg.vector.x = -pose.velocity.z;
        v_msg.vector.y = -pose.velocity.x;
        v_msg.vector.z = pose.velocity.y;
        tf::Vector3 tfv;
        tf::vector3MsgToTF(v_msg.vector,tfv);
        tf::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,v_msg.vector);

        geometry_msgs::Vector3Stamped om_msg;
        om_msg.vector.x = -pose.angular_velocity.z;
        om_msg.vector.y = -pose.angular_velocity.x;
        om_msg.vector.z = pose.angular_velocity.y;
        tf::vector3MsgToTF(om_msg.vector,tfv);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,om_msg.vector);


        nav_msgs::Odometry odom_msg;
        _seq[streamIndex] += 1;

        odom_msg.header.frame_id = this->odomFrameId;
        odom_msg.child_frame_id = this->frameId[POSE];
        odom_msg.header.stamp = t;
        odom_msg.header.seq = _seq[streamIndex];
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        _imu_publishers[streamIndex].publish(odom_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
}*/

/*
//////////////////////////////////////////////////
void BaseRealSenseNode::MultipleMessageCallback(rs2::frame frame, imu_sync_method sync_method)
{
    auto stream = frame.get_profile().stream_type();
    switch (stream)
    {
        case RS2_STREAM_GYRO:
        case RS2_STREAM_ACCEL:
            if (sync_method > imu_sync_method::NONE)
              imu_callback_sync(frame, sync_method);
            else imu_callback(frame);
            break;
        case RS2_STREAM_POSE:
            pose_callback(frame);
            break;
        default:
            frame_callback(frame);
    }
}
*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::SetBaseTime(double _frameTime, bool _warnNoMetadata)
{
    ROS_WARN_COND(warn_no_metadata, "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

    this->timeBase = std::chronot::steady_clock::now();
    this->cameraTimeBase = frame_time;
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::setupStreams()
{
  ROS_INFO("setupStreams...");
    try{
    // Publish image stream info
        for (auto& profiles : this->enabledProfiles)
        {
            for (auto& profile : profiles.second)
            {
                if (profile.is<rs2::video_stream_profile>())
                {
                    auto video_profile = profile.as<rs2::video_stream_profile>();
                    this->UpdateStreamCalibData(video_profile);
                }
            }
        }

        // Streaming IMAGES
        std::map<std::string, std::vector<rs2::stream_profile> > profiles;
        std::map<std::string, rs2::sensor> active_sensors;
        for (const std::pair<StreamIndexPair, std::vector<rs2::stream_profile>>& profile : this->enabledProfiles)
        {
            std::string module_name = this->sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
            ROS_INFO_STREAM("insert " << rs2_stream_to_string(profile.second.begin()->stream_type())
              << " to " << module_name);
            profiles[module_name].insert(profiles[module_name].begin(),
                                            profile.second.begin(),
                                            profile.second.end());
            active_sensors[module_name] = this->sensors[profile.first];
        }

        for (const std::pair<std::string, std::vector<rs2::stream_profile> >& sensor_profile : profiles)
        {
            std::string module_name = sensor_profile.first;
            rs2::sensor sensor = active_sensors[module_name];
            sensor.open(sensor_profile.second);
            sensor.start(_sensors_callback[module_name]);
            if (sensor.is<rs2::depth_sensor>())
            {
                _depth_scale_meters = sensor.as<rs2::depth_sensor>().get_depth_scale();
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::UpdateStreamCalibData(
    const rs2::video_stream_profile &videoProfile)
{
    StreamIndexPair streamIndex{video_profile.stream_type(), video_profile.streamIndex()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[streamIndex] = intrinsic;
    _camera_info[streamIndex].width = intrinsic.width;
    _camera_info[streamIndex].height = intrinsic.height;
    _camera_info[streamIndex].header.frame_id = this->opticalFrameId[streamIndex];

    _camera_info[streamIndex].K.at(0) = intrinsic.fx;
    _camera_info[streamIndex].K.at(2) = intrinsic.ppx;
    _camera_info[streamIndex].K.at(4) = intrinsic.fy;
    _camera_info[streamIndex].K.at(5) = intrinsic.ppy;
    _camera_info[streamIndex].K.at(8) = 1;

    _camera_info[streamIndex].P.at(0) = _camera_info[streamIndex].K.at(0);
    _camera_info[streamIndex].P.at(1) = 0;
    _camera_info[streamIndex].P.at(2) = _camera_info[streamIndex].K.at(2);
    _camera_info[streamIndex].P.at(3) = 0;
    _camera_info[streamIndex].P.at(4) = 0;
    _camera_info[streamIndex].P.at(5) = _camera_info[streamIndex].K.at(4);
    _camera_info[streamIndex].P.at(6) = _camera_info[streamIndex].K.at(5);
    _camera_info[streamIndex].P.at(7) = 0;
    _camera_info[streamIndex].P.at(8) = 0;
    _camera_info[streamIndex].P.at(9) = 0;
    _camera_info[streamIndex].P.at(10) = 1;
    _camera_info[streamIndex].P.at(11) = 0;

    _camera_info[streamIndex].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    _camera_info[streamIndex].R.at(0) = 1.0;
    _camera_info[streamIndex].R.at(1) = 0.0;
    _camera_info[streamIndex].R.at(2) = 0.0;
    _camera_info[streamIndex].R.at(3) = 0.0;
    _camera_info[streamIndex].R.at(4) = 1.0;
    _camera_info[streamIndex].R.at(5) = 0.0;
    _camera_info[streamIndex].R.at(6) = 0.0;
    _camera_info[streamIndex].R.at(7) = 0.0;
    _camera_info[streamIndex].R.at(8) = 1.0;

    _camera_info[streamIndex].D.resize(5);
    for (int i = 0; i < 5; i++)
    {
        _camera_info[streamIndex].D.at(i) = intrinsic.coeffs[i];
    }

    if (streamIndex == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[streamIndex].P.at(3) = 0;     // Tx
        _camera_info[streamIndex].P.at(7) = 0;     // Ty
    }

    if (this->alignDepth)
    {
        for (auto& profiles : this->enabledProfiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                StreamIndexPair streamIndex{video_profile.stream_type(), video_profile.streamIndex()};
                _depth_aligned_camera_info[streamIndex] = _camera_info[streamIndex];
            }
        }
    }
}*/

/*tf::Quaternion BaseRealSenseNode::RotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishStaticTf(const ros::Time& t,
                                          const float3& trans,
                                          const tf::Quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.getX();
    msg.transform.rotation.y = q.getY();
    msg.transform.rotation.z = q.getZ();
    msg.transform.rotation.w = q.getW();
    _static_tf_msgs.push_back(msg);
}*/

/*void BaseRealSenseNode::CalcAndPublishStaticTransform(const StreamIndexPair& stream, const rs2::stream_profile& base_profile)
{
  // Transform base to stream
  tf::Quaternion quaternion_optical;
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  float3 zero_trans{0, 0, 0};

  ros::Time transform_ts_ = ros::Time::now();

  rs2_extrinsics ex;
  try
  {
    ex = this->Profile(stream).get_extrinsics_to(base_profile);
  }
  catch (std::exception& e)
  {
    if (!strcmp(e.what(), "Requested extrinsics are not available!"))
    {
      ROS_WARN_STREAM(e.what() << " : using unity as default.");
      ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
    }
    else
    {
      throw e;
    }
  }

  auto Q = this->RotationMatrixToQuaternion(ex.rotation);
  Q = quaternion_optical * Q * quaternion_optical.inverse();

  float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
  this->PublishStaticTf(transform_ts_, trans, Q, this->baseframeId, _frame_id[stream]);

  // Transform stream frame to stream optical frame
  this->PublishStaticTf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], this->opticalFrameId[stream]);

  if (this->alignDepth && this->depthAlignedFrameId.find(stream) != this->depthAlignedFrameId.end())
  {
    this->PublishStaticTf(transform_ts_, trans, Q, this->baseframeId, this->depthAlignedFrameId[stream]);
    this->PublishStaticTf(transform_ts_, zero_trans, quaternion_optical, this->depthAlignedFrameId[stream], this->opticalFrameId[stream]);
  }
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::SetBaseStream()
{
    const std::vector<StreamIndexPair> base_stream_priority = {DEPTH, POSE};

    std::vector<StreamIndexPair>::const_iterator base_stream(base_stream_priority.begin());
    while( (this->sensors.find(*base_stream) == this->sensors.end()) && (base_stream != base_stream_priority.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_INFO_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_stream = *base_stream;
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishStaticTransforms()
{
  rs2::stream_profile base_profile = this->Profile(_base_stream);

  // Publish static transforms
  if (this->publishTf)
  {
    for (std::pair<StreamIndexPair, bool> ienable : _enable)
    {
      if (ienable.second)
      {
        this->CalcAndPublishStaticTransform(ienable.first, base_profile);
      }
    }
    // Static transform for non-positive values
    if (this->tfPublishRate > 0)
      this->tfThread = std::shared_ptr<std::thread>(new std::thread(boost::bind(&BaseRealSenseNode::PublishDynamicTransforms, this)));
    else
      _static_tf_broadcaster.sendTransform(_static_tf_msgs);
  }

  // Publish Extrinsics Topics:
  if (_enable[DEPTH] &&
      _enable[FISHEYE])
  {
    static const char* frame_id = "depth_to_fisheye_extrinsics";
    const auto& ex = base_profile.get_extrinsics_to(this->Profile(FISHEYE));

    _depth_to_other_extrinsics[FISHEYE] = ex;
    //_depth_to_other_extrinsics_publishers[FISHEYE].publish( this->RsExtrinsicsToMsg(ex, frame_id));
  }

  if (_enable[DEPTH] &&
      _enable[COLOR])
  {
    static const char* frame_id = "depth_to_color_extrinsics";
    const auto& ex = base_profile.get_extrinsics_to(this->Profile(COLOR));
    _depth_to_other_extrinsics[COLOR] = ex;
    //_depth_to_other_extrinsics_publishers[COLOR].publish(this->RsExtrinsicsToMsg(ex, frame_id));
  }

  if (_enable[DEPTH] &&
      _enable[INFRA1])
  {
    static const char* frame_id = "depth_to_infra1_extrinsics";
    const auto& ex = base_profile.get_extrinsics_to(this->Profile(INFRA1));
    _depth_to_other_extrinsics[INFRA1] = ex;
    //_depth_to_other_extrinsics_publishers[INFRA1].publish(this->RsExtrinsicsToMsg(ex, frame_id));
  }

  if (_enable[DEPTH] &&
      _enable[INFRA2])
  {
    static const char* frame_id = "depth_to_infra2_extrinsics";
    const auto& ex = base_profile.get_extrinsics_to(this->Profile(INFRA2));
    _depth_to_other_extrinsics[INFRA2] = ex;
    //_depth_to_other_extrinsics_publishers[INFRA2].publish(this->RsExtrinsicsToMsg(ex, frame_id));
  }
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishDynamicTransforms()
{
  // Publish transforms for the cameras
  ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", this->tfPublishRate);

  ros::Rate loop_rate(this->tfPublishRate);

  while (ros::ok())
  {
    // Update the time stamp for publication
    ros::Time t = ros::Time::now();
    for(auto& msg : _static_tf_msgs)
      msg.header.stamp = t;

    _dynamic_tf_broadcaster.sendTransform(_static_tf_msgs);

    loop_rate.sleep();
  }
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishIntrinsics()
{
  if (this->enable[GYRO])
  {
    this->infoPublisher[GYRO] = this->node.Advertise<IMUInfo>(
        "gyro/imu_info", 1, true);

    // IMUInfo info_msg = this->ImuInfo(GYRO);
    this->infoPublisher[GYRO].Publish(info_msg);
  }

  if (_enable[ACCEL])
  {
    this->infoPublisher[ACCEL] = ithis->node.Advertise<IMUInfo>(
    "accel/imu_info", 1, true);
    // IMUInfo info_msg = this->ImuInfo(ACCEL);
    this->infoPublisher[ACCEL].publish(info_msg);
  }
}*/

/*void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
  size_t i;

  for (i=0; i < n; ++i)
    dst[n-1-i] = src[i];

}*/

/////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishPointCloud(
    rs2::points _pc, const ros::Time &_time, const rs2::frameset &_frameset)
{
    std::vector<NamedFilter>::iterator pc_filter = find_if(_filters.begin(), _filters.end(), [] (NamedFilter s) { return s._name == "pointcloud"; } );
    rs2_stream texture_source_id = static_cast<rs2_stream>(pc_filter->_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };

        texture_frame_itr = find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = pc_filter->_filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    _valid_pc_indices.clear();
    for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
    {
        if (static_cast<float>(vertex->z) > 0)
        {
            float i = static_cast<float>(color_point->u);
            float j = static_cast<float>(color_point->v);
            if (_allow_no_texture_points || (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f))
            {
                _valid_pc_indices.push_back(point_idx);
            }
        }
    }

    _msg_pointcloud.header.stamp = t;
    _msg_pointcloud.header.frame_id = this->opticalFrameId[DEPTH];
    _msg_pointcloud.width = _valid_pc_indices.size();
    _msg_pointcloud.height = 1;
    _msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    vertex = pc.get_vertices();
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        unsigned int prev_idx(0);
        for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
        {
            unsigned int idx_jump(*idx-prev_idx);
            prev_idx = *idx;
            vertex+=idx_jump;
            color_point+=idx_jump;

            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;

            float i(color_point->u);
            float j(color_point->v);
            if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
            {
                color_pixel[0] = i * texture_width;
                color_pixel[1] = j * texture_height;
                int pixx = static_cast<int>(color_pixel[0]);
                int pixy = static_cast<int>(color_pixel[1]);
                int offset = (pixy * texture_width + pixx) * num_colors;
                reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
            }

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_color;
        }
    }
    else
    {
        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        unsigned int prev_idx(0);
        for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
        {
            unsigned int idx_jump(*idx-prev_idx);
            prev_idx = *idx;
            vertex+=idx_jump;

            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;

            ++iter_x; ++iter_y; ++iter_z;
        }
    }
    _pointcloud_publisher.publish(_msg_pointcloud);
}*/

//////////////////////////////////////////////////
/*Extrinsics BaseRealSenseNode::RsExtrinsicsToMsg(
    const rs2_extrinsics &_extrinsics, const std::string &_frameId) const
{
  Extrinsics extrinsicsMsg;
  for (int i = 0; i < 9; ++i)
  {
    extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
    if (i < 3)
      extrinsicsMsg.translation[i] = extrinsics.translation[i];
  }

  extrinsicsMsg.header.frame_id = frame_id;
  return extrinsicsMsg;
}
*/

//////////////////////////////////////////////////
/*rs2::stream_profile BaseRealSenseNode::Profile(const StreamIndexPair &_stream)
{
  const std::vector<rs2::stream_profile> profiles =
    this->sensors[stream].get_stream_profiles();

  return *(std::find_if(profiles.begin(), profiles.end(),
        [&stream] (const rs2::stream_profile &profile)
        {
          return ((profile.stream_type() == stream.first) &&
                  (profile.streamIndex() == stream.second));
        }));
}*/

//////////////////////////////////////////////////
/*IMUInfo BaseRealSenseNode::ImuInfo(const StreamIndexPair &_streamIndex)
{
  IMUInfo info{};

  auto sp = this->enabledProfiles[_streamIndex].front().as<
    rs2::motion_stream_profile>();

  rs2_motion_device_intrinsic imuIntrinsics;

  try
  {
    imuIntrinsics = sp.get_motion_intrinsics();
  }
  catch(const std::runtime_error &ex)
  {
    igndbg << "No Motion Intrinsics available.\n";
    imuIntrinsics = {{{1,0,0,0},{0,1,0,0},{0,0,1,0}}, {0,0,0}, {0,0,0}};
  }

  auto index = 0;
  info.frame_id = this->opticalFrameId[_streamIndex];

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      info.data[index] = imuIntrinsics.data[i][j];
      ++index;
    }

    info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
    info.bias_variances[i] = imuIntrinsics.bias_variances[i];
  }

  return info;
}*/

//////////////////////////////////////////////////
/*void BaseRealSenseNode::PublishFrame(
    rs2::frame _frame, const std::chrono::time_point &_time,
    const StreamIndexPair &_stream,
    std::map<StreamIndexPair, cv::Mat> &_images,
    const std::map<StreamIndexPair, ros::Publisher> &_infoPublishers,
    const std::map<StreamIndexPair,
      ImagePublisherWithFrequencyDiagnostics> &_imagePublishers,
    std::map<StreamIndexPair, int> &_seq,
    std::map<StreamIndexPair, sensor_msgs::CameraInfo> &_cameraInfo,
    const std::map<StreamIndexPair, std::string>& _opticalFrameId,
    const std::map<rs2_stream, std::string>& this->encoding,
    bool _copyDataFromFrame)
{
  igndbg << "publishFrame\n";

  unsigned int width = 0;
  unsigned int height = 0;
  auto bpp = 1;

  if (_frame.is<rs2::video_frame>())
  {
    auto image = _frame.as<rs2::video_frame>();
    width = image.get_width();
    height = image.get_height();
    bpp = image.get_bytes_per_pixel();
  }

  auto &image = _images[_stream];

  if (_copyDataFromFrame)
  {
    if (_images[_stream].size() != cv::Size(width, height))
    {
      image.create(height, width, image.type());
    }
    image.data = (uint8_t*)_frame.get_data();
  }

  if (_frame.is<rs2::depth_frame>())
  {
    image = fix_depth_scale(image, _depth_scaled_image[_stream]);
  }

  ++(_seq[_stream]);
  auto &infoPublisher = _infoPublishers.at(_stream);
  auto &imagePublisher = _imagePublishers.at(_stream);

  if (0 != infoPublisher.getNumSubscribers() ||
      0 != imagePublisher.first.getNumSubscribers())
  {
    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(
        std_msgs::Header(), this->encoding.at(_stream.first), image).toImageMsg();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = _opticalFrameId.at(_stream);
    img->header.stamp = _time;
    img->header.seq = _seq[_stream];

    auto &camInfo = _cameraInfo.at(_stream);

    if (camInfo.width != width)
    {
      this->UpdateStreamCalibData(
          _frame.get_profile().as<rs2::video_stream_profile>());
    }

    camInfo.header.stamp = _time;
    camInfo.header.seq = _seq[_stream];
    infoPublisher.publish(camInfo);

    imagePublisher.first.publish(img);
    imagePublisher.second->update();
    igndbg << f.get_profile().stream_type() << "stream published\n";
  }
}*/

//////////////////////////////////////////////////
/*bool BaseRealSenseNode::EnabledProfile(
    const StreamIndexPair &_streamIndex, rs2::stream_profile &_profile)
{
  // Assuming that all D400 SKUs have depth sensor
  auto profiles = this->enabledProfiles[_streamIndex];
  auto it = std::find_if(profiles.begin(), profiles.end(),
      [&](const rs2::stream_profile& profile)
      { return (profile.stream_type() == _streamIndex.first); });

  if (it == profiles.end())
    return false;

  _profile =  *it;
  return true;
}*/



/*TemperatureDiagnostics::TemperatureDiagnostics(std::string name, std::string serial_no)
{
  _updater.add(name, this, &TemperatureDiagnostics::diagnostics);
  _updater.setHardwareID(serial_no);
}

//////////////////////////////////////////////////
void TemperatureDiagnostics::Diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
        status.summary(0, "OK");
        status.add("Index", _crnt_temp);
}
*/
