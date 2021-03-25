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
    /// # Plugin parameters
    ///
    /// * <publication_hz> : An integer that is the maximum publication
    /// hertz rate.
    ///
    /// * <port> : An integer that is websocket port.
    ///
    /// * <authorization_key> : A key used for authentication. If this is
    /// set, then a connection must provide the matching key using an "auth"
    /// call on the websocket. If the <admin_authorization_key> is set, then
    /// the connection can provide that key.
    ///
    /// * <admin_authorization_key> : An admin key used for authentication. If
    /// this is set, then a connection must provide the matching key using an
    /// "auth" call on the websocket. If the <authorization_key> is set, then
    /// the connection can provide that key.
    ///
    /// * <ssl> : Element that contains SSL configuration. For testing
    ///           purposes you can create self-signed SSL certificates. Run
    ///
    /// ```
    /// openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    /// -keyout server.key -out server.cert
    /// ```
    ///
    ///  Use "localhost" for the  "Common Name" question. If you are testing
    ///  with a browser, first navigate to "https://localhost:<port>" and
    ///  accept the self-signed certificate.
    ///
    ///     * <cert_file>: Child element of <ssl> that contains the path to
    ///                     the SSL certificate file.
    ///     * <private_key_file>: Child element of <ssl> that contains the path
    ///                           to SSL private key file.
    ///
    /// # Websocket Server Interface
    ///
    /// The websocket server listens for incoming requests and sends
    /// messages based on the requests.
    ///
    /// All messages on the websocket, incoming and outgoing, are structured
    /// in a frame that consists of four comma separated components:
    ///     1. `operation`: string,
    ///     2. `topic_name`: string,
    ///     3. `message_type`: string, and
    ///     4. `payload`: serialized data.
    ///
    /// The `operation` component is mandatory and must be one of:
    ///     1. "sub": Subscribe to the topic in the `topic_name` component,
    ///     2. "pub": Publish a message from the Ignition Transport topic in
    ///               the `topic_name` component,
    ///     3. "topics": Get the list of available topics,
    ///     3. "topics-types": Get the list of available topics and their
    ///                        message types, and
    ///     4. "protos": Get a string containing all the protobuf
    ///                  definitions.
    ///
    /// The `topic_name` component is mandatory for the "sub" and "pub"
    /// operations. If present, it must be the name of an Ignition Transport
    /// topic.
    ///
    /// The `message_type` component is mandatory for the "pub" operation. If
    /// present it names the Ignition Message type, such as
    /// "ignition.msgs.Clock".
    ///
    /// The `payload` component is mandatory for the "pub" operation. If
    /// present, it contains a serialized string of an Ignition Message.
    ///
    /// ## Example frames
    ///
    /// 1. Get the list of topics: `topics,,,`
    ///
    /// 2. Get the protobuf definitions: `protos,,,`
    ///
    /// 3. Subscribe to the "/clock" topic: `sub,/clock,,`
    ///
    /// 4. Websocket server publishing data on the "/clock" topic:
    ///    `pub,/clock,ignition.msgs.Clock,<serialized_data>`
    ///
    /// # Example usage
    ///
    /// ## Websocket Server
    ///
    /// 1. Define a launch file by copying the following contents to a file
    ///    called `websocket.ign`.
    ///
    /// <!-- Inform ignition::Launch about the JoyToTwist plugin -->
    ///  <plugin name="ignition::launch::WebsocketServer"
    ///      filename="ignition-launch-joystick0">
    ///
    ///    <!-- Publication Hz -->
    ///    <publication_hz>30</publication_hz>
    /// </plugin>
    ///
    /// 2. Run the launch file
    ///
    /// `ign launch -v 4 websocket.ign`
    ///
    /// 3. Open the [index.html](https://github.com/ignitionrobotics/ign-launch/blob/ign-launch3/plugins/websocket_server/index.html) webpage.
    ///
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

      /// \brief Callback that is triggered when a new connection is
      /// established.
      /// \param[in] _socketId ID of the socket.
      public: void OnConnect(int _socketId);

      /// \brief Callback that is triggered when a connection ended.
      /// \param[in] _socketId ID of the socket.
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

        public: bool authorized{false};
      };

      private: void QueueMessage(Connection *_connection,
                   const char *_data, const size_t _size);

      public: std::mutex mutex;

      /// \brief A mutex used in the OnWebsocketSubscribedMessage
      /// function.
      public: std::mutex subscriptionMutex;

      /// \brief All of the websocket connections.
      public: std::map<int, std::unique_ptr<Connection>> connections;

      /// \brief All of the subscribed Ignition topics.
      /// The key is the topic name, and the value is the set of websocket
      /// connections that have subscribed to the topic.
      public: std::map<std::string, std::set<int>> topicConnections;

      /// \brief Run loop mutex.
      public: std::mutex runMutex;

      /// \brief Run loop condition variable.
      public: std::condition_variable runConditionVariable;

      /// \brief Number of pending messages. This is used to throttle the
      /// run loop.
      public: int messageCount{0};

      /// \brief The maximum number of connections. A negative number
      /// indicates no limit.
      public: int maxConnections{-1};

      /// \brief Time of last publication for each subscribed topic. The key
      /// is the topic name and the value the time of last publication.
      /// \sa publishPeriod.
      private: std::map<std::string,
               std::chrono::time_point<std::chrono::steady_clock>>
                 topicTimestamps;

      /// \brief The set of valid operations. This enum must align with the
      /// `operations` member variable.
      private: enum Operation
               {
                 /// \brief Subscribe to a topic.
                 SUBSCRIBE = 0,

                 /// \brief Publish a message from a topic.
                 PUBLISH = 1,

                 /// \brief Get the list of topics.
                 TOPICS = 2,

                 /// \brief Get the protobuf definitions.
                 PROTOS = 3,
               };

      /// \brief The set of valid operations, in string  form. These values
      /// can be sent in websocket message frames.
      /// These valus must align with the `Operation` enum.
      private: std::vector<std::string> operations{
                 "sub", "pub", "topics", "protos"};

      /// \brief Store publish headers for topics. This is here to improve
      /// performance. Keys are topic names and values are frame headers.
      private: std::map<std::string, std::string> publishHeaders;

      /// \brief Period at which messages will be published on the websocket
      /// for each subscribed topic.
      /// \sa topicTimestamps.
      private: std::chrono::nanoseconds publishPeriod;

      /// \brief Authorization key used to validate a web-socket connection.
      private: std::string authorizationKey;

      /// \brief Administrator authorization key used to validate a web-socket
      /// connection.
      private: std::string adminAuthorizationKey;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::WebsocketServer, ignition::launch::Plugin)

#endif
