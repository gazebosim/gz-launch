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
#ifndef IGNITION_LAUNCH_WEBSOCKETSERVER_HH_
#define IGNITION_LAUNCH_WEBSOCKETSERVER_HH_

#include <list>
#include <map>
#include <memory>
#include <thread>
#include <ignition/launch/Plugin.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/MessageInfo.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Util.hh>
#include <libwebsockets.h>

namespace ignition
{
  namespace launch
  {
    /// \brief Reads from a USB joystick device and outputs
    ///  ignition::msgs::WebsocketServer messages.
    ///
    /// # Example usage
    ///
    /// <!-- Inform ignition::Launch about the JoyToTwist plugin -->
    ///  <plugin name="ignition::launch::WebsocketServer"
    ///      filename="ignition-launch-joystick0">
    ///
    ///    <!-- WebsocketServer device -->
    ///    <device>/dev/input/js0</device>
    ///
    ///    <!-- True enables sticky buttons. -->
    ///    <sticky_buttons>false</sticky_buttons>
    ///
    ///    <!-- WebsocketServer deadzone -->
    ///    <dead_zone>0.05</dead_zone>
    ///
    ///    <!-- Update rate -->
    ///    <rate>60</rate>
    ///    <accumulation_rate>1000</accumulation_rate>
    /// </plugin>

    class WebsocketServer : public ignition::launch::Plugin
    {
      /// \brief Constructor
      public: WebsocketServer();

      /// \brief Destructor
      public: virtual ~WebsocketServer();

      // Documentation inherited
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      public: void Run();

      private: void OnWebsocketSubscribedMessage(const char *_data,
                   const size_t _size,
                   const ignition::transport::MessageInfo &_info);

      public: void OnConnect(int _socketId);
      public: void OnDisconnect(int _socketId);
      public: void OnMessage(int _socketId, const std::string &_msg);

      public: void OnRequestMessage(int _socketId, const std::string &_msg);


      private: ignition::transport::Node node;

      private: bool run = true;
      private: std::thread *thread = nullptr;
      private: struct lws_context *context = nullptr;

      private: std::vector<struct lws_protocols> protocols;
      private: class Connection
      {
        public: std::chrono::system_clock::time_point creationTime;
        public: std::list<std::unique_ptr<char>> buffer;
        public: std::list<int> len;
        public: std::mutex mutex;
      };

      private: void QueueMessage(Connection *_connection,
                   const char *_data, const size_t _size);

      public: std::mutex mutex;
      public: std::map<int, std::unique_ptr<Connection>> connections;
      public: std::map<std::string, std::set<int>> topicConnections;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::WebsocketServer, ignition::launch::Plugin)

#endif
