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
#ifndef UVC_CAMERA_HH_
#define UVC_CAMERA_HH_

#include <string>
#include <linux/videodev2.h>
#include <stdint.h>

namespace UVCCamera
{
  enum v4l2_uvc_exposure_auto_type
  {
    V4L2_UVC_EXPOSURE_MANUAL = 1,
    V4L2_UVC_EXPOSURE_AUTO = 2,
    V4L2_UVC_EXPOSURE_SHUTTER_PRIORITY = 4,
    V4L2_UVC_EXPOSURE_APERTURE_PRIORITY = 8
  };

  static const int exp_vals[] =
  {
    V4L2_UVC_EXPOSURE_MANUAL,
    V4L2_UVC_EXPOSURE_AUTO,
    V4L2_UVC_EXPOSURE_SHUTTER_PRIORITY,
    V4L2_UVC_EXPOSURE_APERTURE_PRIORITY
  };

  #define CSU32 const static uint32_t

  class Camera
  {
    public: enum mode_t
            {
              MODE_RGB,
              MODE_MJPG,
              MODE_YUYV
            } mode;

    /// \brief Constructor
    public: Camera(const char *device, mode_t _mode = MODE_RGB,
                int _width = 640, int _height = 480, int _fps = 30);

    /// \brief Destructor
    public: ~Camera();

    /// \brief
    public: static void Enumerate();

    /// \brief
    public: int Grab(unsigned char **_frame, uint32_t &_bytesUsed);

    /// \brief
    public: void Release(unsigned _bufIdx);

    /// \brief
    public: bool SetAutoWhiteBalance(bool _on);

    /// \brief
    public: void SetMotionThresholds(int _lum, int _count);

    /// \brief
    public: bool SetControl(uint32_t _id, int _val);

    /// \brief
    private: std::string device;

    /// \brief
    private: int fd;

    /// \brief
    private: int motionThresholdLuminance;

    /// \brief
    private: int motionThresholdCount;

    /// \brief
    private: unsigned int width;

    /// \brief
    private: unsigned int height;

    /// \brief
    private: unsigned int fps;

    /// \brief
    private: v4l2_format fmt;

    /// \brief
    private: v4l2_capability cap;

    /// \brief
    private: v4l2_streamparm streamParm;

    /// \brief
    private: v4l2_requestbuffers requestBuffer;

    /// \brief
    private: v4l2_buffer buf;

    /// \brief
    private: v4l2_timecode timeCode;

    /// \brief
    private: static const unsigned int NUM_BUFFER = 2;

    /// \brief
    private: void *mem[NUM_BUFFER];

    /// \brief
    private: unsigned bufLength;

    /// \brief
    private: unsigned char *rgbFrame;

    /// \brief
    private: unsigned char *lastYUVFrame;

    //------------------------- new camera class controls ---------------------
    CSU32 V4L2_CTRL_CLASS_USER_NEW = 0x00980000;
    CSU32 V4L2_CID_BASE_NEW = V4L2_CTRL_CLASS_USER_NEW | 0x900;
    CSU32 V4L2_CID_POWER_LINE_FREQUENCY_NEW = V4L2_CID_BASE_NEW + 24;
    CSU32 V4L2_CID_HUE_AUTO_NEW = V4L2_CID_BASE_NEW + 25;
    CSU32 V4L2_CID_WHITE_BALANCE_TEMPERATURE_NEW = V4L2_CID_BASE_NEW + 26;
    CSU32 V4L2_CID_SHARPNESS_NEW = V4L2_CID_BASE_NEW + 27;
    CSU32 V4L2_CID_BACKLIGHT_COMPENSATION_NEW = V4L2_CID_BASE_NEW + 28;
    CSU32 V4L2_CID_LAST_NEW = V4L2_CID_BASE_NEW + 29;

    // Camera class controls
    CSU32 V4L2_CTRL_CLASS_CAMERA_NEW = 0x009A0000;
    CSU32 V4L2_CID_CAMERA_CLASS_BASE_NEW = V4L2_CTRL_CLASS_CAMERA_NEW | 0x900;
    CSU32 V4L2_CID_EXPOSURE_AUTO_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 1;
    CSU32 V4L2_CID_EXPOSURE_ABSOLUTE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 2;
    CSU32 V4L2_CID_EXPOSURE_AUTO_PRIORITY_NEW =
      V4L2_CID_CAMERA_CLASS_BASE_NEW+3;
    CSU32 V4L2_CID_PAN_RELATIVE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 4;
    CSU32 V4L2_CID_TILT_RELATIVE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 5;
    CSU32 V4L2_CID_PAN_RESET_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 6;
    CSU32 V4L2_CID_TILT_RESET_NEW  = V4L2_CID_CAMERA_CLASS_BASE_NEW + 7;
    CSU32 V4L2_CID_PAN_ABSOLUTE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 8;
    CSU32 V4L2_CID_TILT_ABSOLUTE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 9;
    CSU32 V4L2_CID_FOCUS_ABSOLUTE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 10;
    CSU32 V4L2_CID_FOCUS_RELATIVE_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 11;
    CSU32 V4L2_CID_FOCUS_AUTO_NEW = V4L2_CID_CAMERA_CLASS_BASE_NEW + 12;
    CSU32 V4L2_CID_CAMERA_CLASS_LAST = V4L2_CID_CAMERA_CLASS_BASE_NEW + 13;

    //--------------- old private class controls ------------------------------
    CSU32 V4L2_CID_PRIVATE_BASE_OLD = 0x08000000;
    CSU32 V4L2_CID_BACKLIGHT_COMPENSATION_OLD = V4L2_CID_PRIVATE_BASE_OLD + 0;
    CSU32 V4L2_CID_POWER_LINE_FREQUENCY_OLD = V4L2_CID_PRIVATE_BASE_OLD + 1;
    CSU32 V4L2_CID_SHARPNESS_OLD = V4L2_CID_PRIVATE_BASE_OLD + 2;
    CSU32 V4L2_CID_HUE_AUTO_OLD = V4L2_CID_PRIVATE_BASE_OLD + 3;
    CSU32 V4L2_CID_FOCUS_AUTO_OLD = V4L2_CID_PRIVATE_BASE_OLD + 4;
    CSU32 V4L2_CID_FOCUS_ABSOLUTE_OLD = V4L2_CID_PRIVATE_BASE_OLD + 5;
    CSU32 V4L2_CID_FOCUS_RELATIVE_OLD = V4L2_CID_PRIVATE_BASE_OLD + 6;
    CSU32 V4L2_CID_PAN_RELATIVE_OLD = V4L2_CID_PRIVATE_BASE_OLD + 7;
    CSU32 V4L2_CID_TILT_RELATIVE_OLD = V4L2_CID_PRIVATE_BASE_OLD + 8;
    CSU32 V4L2_CID_PANTILT_RESET_OLD = V4L2_CID_PRIVATE_BASE_OLD + 9;
    CSU32 V4L2_CID_EXPOSURE_AUTO_OLD = V4L2_CID_PRIVATE_BASE_OLD + 10;
    CSU32 V4L2_CID_EXPOSURE_ABSOLUTE_OLD = V4L2_CID_PRIVATE_BASE_OLD + 11;
    CSU32 V4L2_CID_WHITE_BALANCE_TEMP_AUTO_OLD  = V4L2_CID_PRIVATE_BASE_OLD+12;
    CSU32 V4L2_CID_WHITE_BALANCE_TEMP_OLD = V4L2_CID_PRIVATE_BASE_OLD + 13;
    CSU32 V4L2_CID_PRIVATE_LAST = V4L2_CID_WHITE_BALANCE_TEMP_OLD + 1;
    // dynamic controls
    CSU32 UVC_CTRL_DATA_TYPE_RAW = 0;
    CSU32 UVC_CTRL_DATA_TYPE_SIGNED  = 1;
    CSU32 UVC_CTRL_DATA_TYPE_UNSIGNED  = 2;
    CSU32 UVC_CTRL_DATA_TYPE_BOOLEAN = 3;
    CSU32 UVC_CTRL_DATA_TYPE_ENUM = 4;
    CSU32 UVC_CTRL_DATA_TYPE_BITMASK = 5;
    CSU32 V4L2_CID_BASE_EXTCTR = 0x0A046D01;
    CSU32 V4L2_CID_BASE_LOGITECH = V4L2_CID_BASE_EXTCTR;
    //CSU32 V4L2_CID_PAN_RELATIVE_LOGITECH = V4L2_CID_BASE_LOGITECH;
    //CSU32 V4L2_CID_TILT_RELATIVE_LOGITECH = V4L2_CID_BASE_LOGITECH + 1;
    CSU32 V4L2_CID_PANTILT_RESET_LOGITECH = V4L2_CID_BASE_LOGITECH + 2;
    CSU32 V4L2_CID_FOCUS_LOGITECH = V4L2_CID_BASE_LOGITECH + 3;
    CSU32 V4L2_CID_LED1_MODE_LOGITECH = V4L2_CID_BASE_LOGITECH + 4;
    CSU32 V4L2_CID_LED1_FREQUENCY_LOGITECH = V4L2_CID_BASE_LOGITECH + 5;
    CSU32 V4L2_CID_DISABLE_PROCESSING_LOGITECH = V4L2_CID_BASE_LOGITECH + 0x70;
    CSU32 V4L2_CID_RAW_BITS_PER_PIXEL_LOGITECH = V4L2_CID_BASE_LOGITECH + 0x71;
    CSU32 V4L2_CID_LAST_EXTCTR = V4L2_CID_RAW_BITS_PER_PIXEL_LOGITECH;
  };
}
#endif
