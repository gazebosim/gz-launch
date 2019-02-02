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

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "Joystick.hh"

using namespace ignition;

/////////////////////////////////////////////////
Joystick::Joystick()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
Joystick::~Joystick()
{
  if (this->joyThread && this->run)
  {
    this->run = false;
    this->joyThread->join();
  }
  this->joyThread = nullptr;
}

/////////////////////////////////////////////////
void Joystick::Shutdown()
{
  this->run = false;
  if (this->joyThread)
    this->joyThread->join();
}

/////////////////////////////////////////////////
void Joystick::Load(std::map<std::string, std::string> /*_params*/)
{
  // Get the name of the joystick device.
  std::string deviceFilename = "/dev/input/js0";

  bool opened = false;

  // Attempt to open the joystick
  for (int i = 0; i < 10 && !opened; ++i)
  {
    this->joyFd = open(deviceFilename.c_str(), O_RDONLY);

    if (this->joyFd != -1)
    {
      // Close and open the device to get a better initial state.
      close(this->joyFd);
      this->joyFd = open(deviceFilename.c_str(), O_RDONLY);
      opened = true;
    }
    else
    {
      std::cout << "Unable to open joystick at [" << deviceFilename
        << "] Attemping again\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // Stop if we couldn't open the joystick after N attempts
  if (this->joyFd == -1)
  {
    std::cerr << "Unable to open joystick at [" << deviceFilename
          << "]. The joystick will not work.\n";
    return;
  }

  // auto stickyButtons = joy->Get<bool>("sticky_buttons", false).first;

  // Read the amount of dead zone for the analog joystick
  float deadzone = 0.05;
  /*float deadzone = ignition::math::clamp(
      joy->Get<float>("dead_zone", 0.05f).first,
      0.0f, 0.9f);
      */

  // Read the rate at which data should be published
  //float intervalRate = joy->Get<float>("rate", 1.0f).first;
  float intervalRate = 60.0f;
  if (intervalRate <= 0)
    this->interval = 1.0f;
  else
    this->interval = 1.0f / intervalRate;

  // Read the rate at which joystick data should be accumulated into
  // a message.
  //float accumulationRate = joy->Get<float>("accumulation_rate", 1000).first;
  float accumulationRate = 1000;
  if (accumulationRate <= 0)
    this->accumulationInterval = 0.0f;
  else
    this->accumulationInterval = 1.0f / accumulationRate;

  // Check that we are not publishing faster than accumulating data. This is
  // not a critical error, but doesn't make a whole lot of sense.
  if (this->interval < this->accumulationInterval)
  {
    std::cout << "The publication rate of [" << 1.0 / this->interval
      << " Hz] is greater than the accumulation rate of ["
      << 1.0 / this->accumulationInterval
      << " Hz]. Timing behavior is ill defined.\n";
  }

  this->unscaledDeadzone = 32767.0f * deadzone;
  this->axisScale = -1.0f / (1.0f - deadzone) / 32767.0f;

  //auto topic = joy->Get<std::string>("topic", "/joy").first;

  // Create the publisher of joy messages
  this->pub = this->node.Advertise<ignition::msgs::Joy>("/joy");

  this->run = true;
  this->joyThread = new std::thread(std::bind(&Joystick::Run, this));
}

//////////////////////////////////////////////////
void Joystick::Run()
{
  fd_set set;
  struct timeval tv;
  bool timeoutSet = false;
  bool accumulate = false;
  bool accumulating = false;

  ignition::msgs::Joy joyMsg;
  ignition::msgs::Joy lastJoyMsg;
  ignition::msgs::Joy stickyButtonsJoyMsg;

  while (this->run)
  {
    FD_ZERO(&set);
    FD_SET(this->joyFd, &set);

    int selectOut = select(this->joyFd+1, &set, NULL, NULL, &tv);

    if (selectOut == -1)
    {
      tv.tv_sec = 0;
      tv.tv_usec = 0;

      std::cout << "Joystick might be closed\n";
      if (this->run)
        continue;
      else
        break;
    }
    else if (!this->run)
      break;

    js_event event;

    if (FD_ISSET(this->joyFd, &set))
    {
      if (read(this->joyFd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
      {
        std::cout << "Joystick read failed, might be closed\n";
        return;
      }

      float value = event.value;
      switch (event.type)
      {
        case JS_EVENT_BUTTON:
        case JS_EVENT_BUTTON | JS_EVENT_INIT:
          {
            // Update number of buttons
            if (event.number >= joyMsg.buttons_size())
            {
              joyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              lastJoyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              stickyButtonsJoyMsg.mutable_buttons()->Resize(
                  event.number+1, 0.0f);
            }

            // Update the button
            joyMsg.set_buttons(event.number,
                !ignition::math::equal(value, 0.0f) ? 1 : 0);

            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            accumulate = !(event.type & JS_EVENT_INIT);
            break;
          }
        case JS_EVENT_AXIS:
        case JS_EVENT_AXIS | JS_EVENT_INIT:
          {
            if (event.number >= joyMsg.axes_size())
            {
              joyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              lastJoyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              stickyButtonsJoyMsg.mutable_axes()->Resize(
                  event.number+1, 0.0f);
            }

            // Smooth the deadzone
            if (value < -this->unscaledDeadzone)
              value += this->unscaledDeadzone;
            else if (value > this->unscaledDeadzone)
              value -= this->unscaledDeadzone;
            else
              value = 0.0f;

            joyMsg.set_axes(event.number, value * this->axisScale);

            // Will wait a bit before sending to try to combine events.
            accumulate = true;
            break;
          }
        default:
          {
            std::cout << "Unknown event type: time[" << event.time << "] "
              << "value[" << value << "] "
              << "type[" << event.type << "h] "
              << "number["<< event.number << "]" << std::endl;
            break;
          }
       }
     }
     // Assume that the timer has expired.
     else if (timeoutSet)
       accumulate = false;

    if (!accumulate)
    {
      if (this->stickyButtons)
      {
        // process each button
        for (int i = 0; i < joyMsg.buttons_size(); ++i)
        {
          // change button state only on transition from 0 to 1
          if (joyMsg.buttons(i) == 1 && lastJoyMsg.buttons(i) == 0)
          {
            stickyButtonsJoyMsg.set_buttons(i,
              stickyButtonsJoyMsg.buttons(i) ? 0 : 1);
          }
        }

        // update last published message
        lastJoyMsg = joyMsg;

        // Copy the axis
        stickyButtonsJoyMsg.mutable_axes()->CopyFrom(joyMsg.axes());

        // Publish the stick buttons message
        this->pub.Publish(stickyButtonsJoyMsg);
      }
      else
      {
        // Publish the normal joy message
        this->pub.Publish(joyMsg);
      }

      timeoutSet = false;
      accumulating = false;
      accumulate = false;
    }

    // If an axis event occurred, start a timer to combine with other events.
    if (!accumulating && accumulate)
    {
      tv.tv_sec = trunc(this->accumulationInterval);
      tv.tv_usec = (this->accumulationInterval - tv.tv_sec) * 1e6;
      accumulating = true;
      timeoutSet = true;
    }

    // Set a timeout for the signal call at the beginning of this loop.
    if (!timeoutSet)
    {
      tv.tv_sec = trunc(this->interval);
      tv.tv_usec = (this->interval - tv.tv_sec) * 1e6;
      timeoutSet = true;
    }
   }

  // Close the joystick
  close(this->joyFd);
}
