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
#ifndef IGNITION_LAUNCH_CAMERA_PLUGIN_HH_
#define IGNITION_LAUNCH_CAMERA_PLUGIN_HH_

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "UVCCamera.hh"
#include "ignition/launch/Plugin.hh"

using namespace ignition::launch;

namespace ignition
{
  class Camera : public ignition::launch::Plugin
  {
    public: Camera();
    public: virtual ~Camera();
    public: virtual void Load(
                std::map<std::string, std::string> _params) override final;
    public: virtual void Shutdown() override final;

    private: void FeedImages();

    private: bool run = false;
    private: int width = 640;
    private: int height = 480;
    private: int fps = 10;
    private: int skipFrames = 0;
    private: int framesToSkip = 0;
    private: std::string device = "/dev/video0";
    private: std::string frame = "camera";
    private: bool rotate = false;

    private: UVCCamera::Camera *uvcCam = nullptr;
    private: std::thread *imageThread = nullptr;

    private: ignition::transport::Node node;
    private: ignition::transport::Node::Publisher imgPub;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(Camera, ignition::launch::Plugin)

#endif
