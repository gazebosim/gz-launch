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
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>

#include "Camera.hh"

using namespace ignition;

/////////////////////////////////////////////////
Camera::Camera()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
Camera::~Camera()
{
  if (this->imageThread && this->run)
  {
    this->run = false;
    this->imageThread->join();
  }
  this->imageThread = nullptr;

  if (this->uvcCam)
    delete this->uvcCam;
}

/////////////////////////////////////////////////
void Camera::Load(std::map<std::string, std::string> _params)
{
  std::string outputTopic = "/image";

  // Get the camera device name
  if (_params.find("device") != _params.end())
    this->device = _params["device"];

  // Get the output topic name
  if (_params.find("output_topic") != _params.end())
    outputTopic = _params["output_topic"];

  // Get width value
  if (_params.find("width") != _params.end())
    this->width = std::stoi(_params["width"]);

  // Get height value
  if (_params.find("height") != _params.end())
    this->height = std::stoi(_params["height"]);

  // Create the image publisher
  this->imgPub =
    this->node.Advertise<ignition::msgs::Image>(outputTopic);

  // initialize the camera
  try
  {
    this->uvcCam = new UVCCamera::Camera(this->device.c_str(),
        UVCCamera::Camera::MODE_RGB, this->width, this->height, this->fps);
  }
  catch(std::runtime_error &_e)
  {
    ignerr << "Unable to create camera due to exception[" << _e.what() << "]\n";
    ignerr << "Another action or zombie process may be using the camera.\n";
    return;
  }

  // this->uvcCam->SetMotionThresholds(100, -1);

  // and turn on the streamer
  this->run = true;
  this->imageThread = new std::thread(std::bind(&Camera::FeedImages, this));
}

/////////////////////////////////////////////////
void Camera::Shutdown()
{
  this->run = false;
  if (this->imageThread)
    this->imageThread->join();
}

/////////////////////////////////////////////////
void Camera::FeedImages()
{
  unsigned int pairID = 0;
  while (this->run)
  {
    unsigned char *imgFrame = NULL;
    uint32_t bytesUsed;

    auto captureTime = std::chrono::system_clock::now();

    int idx = this->uvcCam->Grab(&imgFrame, bytesUsed);

    // Read in every frame the camera generates, but only send each
    // (skipFrames + 1)th frame. It's set up this way just because
    // this is based on Stereo...
    if (this->skipFrames == 0 || this->framesToSkip == 0)
    {
      if (imgFrame)
      {
        ignition::msgs::Image image;
        // image.set_frame(this->frame);
        // image.set_sequence(pairID);

        auto sec = std::chrono::time_point_cast<std::chrono::seconds>(
            captureTime);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            captureTime - sec);

        image.mutable_header()->mutable_stamp()->set_sec(
            sec.time_since_epoch().count());
        image.mutable_header()->mutable_stamp()->set_nsec(nsec.count());
        image.set_width(this->width);
        image.set_height(this->height);
        image.set_step(3 * this->width);

        /// TODO: fix pixel format
        image.set_pixel_format(common::Image::RGB_INT8);

        image.set_data(imgFrame, this->width * this-> height * 3);

        this->imgPub.Publish(image);

        ++pairID;
      }

      this->framesToSkip = this->skipFrames;
    }
    else
    {
      --this->framesToSkip;
    }

    if (imgFrame)
      this->uvcCam->Release(idx);
  }
  this->run = false;
}
