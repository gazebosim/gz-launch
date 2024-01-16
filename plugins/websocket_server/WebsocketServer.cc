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

#include <algorithm>
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <ignition/msgs.hh>

#include "MessageDefinitions.hh"
#include "WebsocketServer.hh"

using namespace gz::launch;

/// \brief Construct a websocket frame header.
/// \param[in] _op The operation string.
/// \param[in] _topic The topic name string.
/// \param[in] _type The message type string.
/// \return A string that is the frame header.
#define BUILD_HEADER(_op, _topic, _type) ((_op)+","+(_topic)+","+(_type)+",")

/// \brief Construction a complete websocket frame.
/// \param[in] _op The operation string.
/// \param[in] _topic The topic name string.
/// \param[in] _type The message type string.
/// \param[in] _payload The complete payload string.
/// \return A string that is the frame header.
#define BUILD_MSG(_op, _topic, _type, _payload) (BUILD_HEADER(_op, _topic, _type) + _payload)

/// \brief Gets the websocket server from the lws connection passed to a
/// handler.
/// \attention The protocol should be defined with a reference to a websocket
/// server in the `user` field.
/// \param[in] _wsi lws connection.
/// \return A pointer to the websocket server assigned to the protocol.
WebsocketServer *get_server(struct lws *_wsi)
{
  WebsocketServer *self = nullptr;

  // Get the protocol definition for this callback
  lws_protocols *protocol = const_cast<lws_protocols *>(lws_get_protocol(_wsi));

  // It's possible that the protocol is null.
  if (protocol)
    self = static_cast<WebsocketServer *>(protocol->user);

  return self;
}

/// \brief Sets HTTP response status code and writes Content-Type and
/// Content-Length HTTP headers.
/// \param[in] _wsi lws connection.
/// \param[in] _statusCode Status code.
/// \param[in] _contentType Content mime-type.
/// \param[in] _contentLength Size of the body in bytes.
/// \return Returns 1 if there was an error writing the header, 0 otherwise.
int write_http_headers(struct lws *_wsi,
                       int _statusCode,
                       const char *_contentType,
                       unsigned long _contentLength)
{
  // Buffer is oversized to account for variable content lengths and future
  // potential headers.
  unsigned char buf[4096 + LWS_PRE];
  unsigned char *p, *start, *end;
  int n;

  p = buf + LWS_PRE;
  start = p;
  end = p + sizeof(buf) - LWS_PRE;

  // Status code
  if (lws_add_http_header_status(_wsi,
      _statusCode,
      reinterpret_cast<unsigned char **>(&p),
      end))
    return 1;

  // Content-Type
  if (lws_add_http_header_by_token(_wsi,
      WSI_TOKEN_HTTP_CONTENT_TYPE,
      reinterpret_cast<const unsigned char *>(_contentType),
      strlen(_contentType),
      &p,
      end))
    return 1;

  // Content-Length
  if (lws_add_http_header_content_length(_wsi,
      _contentLength - 1,
      &p,
      end))
    return 1;

  // Finalize header
  if (lws_finalize_http_header(_wsi, &p, end))
    return 1;

  // Write headers
  n = lws_write(_wsi, start, p - start, LWS_WRITE_HTTP_HEADERS);
  if (n < 0)
    return 1;

  return 0;
}

/// \brief Handles HTTP lws events.
/// \note This is called by rootCallback to handle HTTP-specific events.
/// rootCallback is called first because regular HTTP requests do not provide a
/// protocol name and the request is sent to rootCallback by default.
/// \param _wsi lws connection.
/// \param _reason lws event. Reason for the call.
/// \param _user Pointer to per-session user data allocated by library.
/// \param _in Pointer used for some callback reasons.
/// \param _len Length set for some callback reasons.
/// \return Returns 1 if there an error was found while processing an event,
/// or -1 otherwise to signal lws to close the request.
int httpCallback(struct lws *_wsi,
                 enum lws_callback_reasons _reason,
                 void *_user,
                 void *_in,
                 size_t _len)
{
  WebsocketServer *self = get_server(_wsi);

  switch (_reason)
  {
    case LWS_CALLBACK_HTTP:
    {
      char *URI = (char *) _in;
      igndbg << "Requested URI: " << URI << "\n";

      // Router
      // Server metrics
      if (strcmp(URI, "/metrics") == 0)
      {
        igndbg << "Handling /metrics\n";

        // TODO Support a proper way to output metrics

        // Format contains the format of the string returned by this route.
        // The following metrics are currently supported:
        // * connections - Number of live connections.
        const char *format = "{ \"connections\": %s }";

        // Get number of connections
        std::string conns = std::to_string(self->connections.size());

        // Prepare the output
        size_t buflen = strlen(format) + (conns.size() - 1);
        unsigned char buf[buflen + LWS_PRE];
        int n;
        n = snprintf(reinterpret_cast<char *>(buf), buflen, format,
            conns.c_str());
        // Check that no characters were discarded
        if (n - int(buflen) > 0)
        {
          ignwarn << "Discarded "
            << n - int(buflen)
            << "characters when preparing metrics.\n";
        }

        // Write response headers
        if (write_http_headers(_wsi, 200, "application/json", buflen))
          return 1;

        // Write response body
        lws_write_http(_wsi,
                       reinterpret_cast<unsigned char *>(buf),
                       strlen(reinterpret_cast<const char *>(buf)));
        break;
      }
      // Return a 404 if no route was matched
      else
      {
        igndbg << "Resource not found.\n";
        lws_return_http_status(_wsi, HTTP_STATUS_NOT_FOUND, "Not Found");
      }
      break;
    }

    default:
      // Do nothing on default.
      break;
  }

  return -1;
}

/// \brief Default request event handler. All requests that do not explicitly
/// specify a protocol name are handled by this function.
/// \param _wsi lws connection.
/// \param _reason lws event. Reason for the call.
/// \param _user Pointer to per-session user data allocated by library.
/// \param _in Pointer used for some callback reasons.
/// \param _len Length set for some callback reasons.
/// \return Returns 1 if there an error was found while processing an event,
/// -1 to signal lws to close the request or 0 to continue processing the
/// request.
int rootCallback(struct lws *_wsi,
                 enum lws_callback_reasons _reason,
                 void *_user,
                 void *_in,
                 size_t _len)
{
  WebsocketServer *self = get_server(_wsi);

  // We require the self pointer, and ignore the cases when this function is
  // called without a self pointer.
  if (!self)
    return 0;

  int fd = lws_get_socket_fd(_wsi);

  // std::lock_guard<std::mutex> mainLock(self->mutex);
  switch (_reason)
  {
    // Filter network connections.
    case LWS_CALLBACK_FILTER_NETWORK_CONNECTION:
      // Prevent too many connections.
      if (self->maxConnections >= 0 &&
          self->connections.size()+1 > self->maxConnections)
      {
        ignerr << "Skipping new connection, limit of "
          << self->maxConnections << " has been reached\n";
        // Return non-zero to close the connection.
        return -1;
      }
      break;

    // Open connections.
    case LWS_CALLBACK_ESTABLISHED:
      igndbg << "LWS_CALLBACK_ESTABLISHED\n";
      self->OnConnect(fd);
      // This will generate a LWS_CALLBACK_SERVER_WRITEABLE event when the
      // connection is writable.
      lws_callback_on_writable(_wsi);
      break;

    // Close connections.
    case LWS_CALLBACK_CLOSED:
      igndbg << "LWS_CALLBACK_CLOSED\n";
      self->OnDisconnect(fd);
      break;

    case LWS_CALLBACK_HTTP:
      igndbg << "LWS_CALLBACK_HTTP\n";
      return httpCallback(_wsi, _reason, _user, _in, _len);
      break;

    // Publish outboud messages
    case LWS_CALLBACK_SERVER_WRITEABLE:
      {
        std::lock_guard<std::mutex> lock(self->connections[fd]->mutex);
        if (!self->connections[fd]->buffer.empty())
        {
          int msgSize = self->connections[fd]->len.front();
          int charsSent = lws_write(_wsi,
              reinterpret_cast<unsigned char *>(
                self->connections[fd]->buffer.front().get() + LWS_PRE),
                msgSize,
              LWS_WRITE_BINARY);

          if (charsSent < msgSize)
          {
            ignerr << "Error writing to socket\n";
          }
          else
          {
            std::scoped_lock<std::mutex> runLock(self->runMutex);
            self->messageCount--;
            // Only pop the message if it was sent successfully.
            self->connections[fd]->buffer.pop_front();
            self->connections[fd]->len.pop_front();
          }
        }

        // This will generate a LWS_CALLBACK_SERVER_WRITEABLE event when the
        // connection is writable.
        lws_callback_on_writable(_wsi);
        break;
      }

    // Handle incoming messages
    case LWS_CALLBACK_RECEIVE:
      igndbg << "LWS_CALLBACK_RECEIVE\n";
      self->OnMessage(fd, std::string((const char *)_in));
      break;

    default:
      // Do nothing on default.
      break;
  }

  return 0;
}

/////////////////////////////////////////////////
WebsocketServer::WebsocketServer()
  : gz::launch::Plugin()
{
}

/////////////////////////////////////////////////
WebsocketServer::~WebsocketServer()
{
  if (this->thread)
  {
    {
      std::scoped_lock<std::mutex> lock(this->runMutex);
      if (this->run)
      {
        this->run = false;
        this->runConditionVariable.notify_all();
      }
    }
    this->thread->join();
  }
  this->thread = nullptr;

  if (this->context)
    lws_context_destroy(this->context);
}

/////////////////////////////////////////////////
bool WebsocketServer::Load(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *elem;

  // Read the publication hertz.
  elem = _elem->FirstChildElement("publication_hz");
  double hz = 60;
  if (elem)
  {
    try
    {
      hz = std::stod(elem->GetText());
    }
    catch (...)
    {
      ignerr << "Unable to convert <publication_hz>" << elem->GetText()
        << "</publication_hz> to a double. Default hz of "
        << hz << " will be used.\n";
    }
  }
  this->publishPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));

  // Get the authorization key, if present.
  elem = _elem->FirstChildElement("authorization_key");
  if (elem)
  {
    const char *txt = elem->GetText();
    if (txt != nullptr)
      this->authorizationKey = txt;
  }

  // Get the admin authorization key, if present.
  elem = _elem->FirstChildElement("admin_authorization_key");
  if (elem)
  {
    const char *txt = elem->GetText();
    if (txt != nullptr)
      this->adminAuthorizationKey = txt;
  }

  int port = 9002;
  // Get the port, if present.
  elem = _elem->FirstChildElement("port");
  if (elem)
  {
    try
    {
      port = std::stoi(elem->GetText());
    }
    catch (...)
    {
      ignerr << "Failed to convert port[" << elem->GetText() << "] to integer."
        << std::endl;
    }
  }
  igndbg << "Using port[" << port << "]\n";

  // Get the maximum connection count, if present.
  elem = _elem->FirstChildElement("max_connections");
  if (elem)
  {
    try
    {
      this->maxConnections = std::stoi(elem->GetText());
    }
    catch (...)
    {
      ignerr << "Failed to convert port[" << elem->GetText() << "] to integer."
        << std::endl;
    }
    igndbg << "Using maximum connection count of "
      << this->maxConnections << std::endl;
  }

  std::string sslCertFile = "";
  std::string sslPrivateKeyFile = "";
  elem = _elem->FirstChildElement("ssl");
  if (elem)
  {
    // Get the ssl cert file, if present.
    const tinyxml2::XMLElement *certElem =
      elem->FirstChildElement("cert_file");
    if (certElem && certElem->GetText())
      sslCertFile = certElem->GetText();

    // Get the ssl private key file, if present.
    const tinyxml2::XMLElement *keyElem =
      elem->FirstChildElement("private_key_file");
    if (keyElem && keyElem->GetText())
      sslPrivateKeyFile = keyElem->GetText();
  }

 // All of the protocols handled by this server.
  this->protocols.push_back(
    {
      // Name of the protocol. This must match the one given in the client
      // Javascript. Javascript example: `new Websocket(url, 'protocol')`.
      // Leave this as "" for the main/default protocol.
      "",
      // The protocol callback.
      rootCallback,
      // Per-session data size.
      0,
      // RX buffer size. Use 0 for unlimited buffer size.
      0,
      // ID, ignored by lws, but useful to contain user information bound to
      // a protocol. This is accessed in the callback via _wsi->protocol->id.
      0,
      // User provided context data. Accessible in the callback via
      // lws_get_protocol(wsi)->user.
      this
    });

  // The terminator
  this->protocols.push_back({NULL, NULL, 0, 0, 0, 0 });

  // We will handle logging
  lws_set_log_level( 0, lwsl_emit_syslog);

  struct lws_context_creation_info info;
  memset(&info, 0, sizeof info);
  info.port = port;
  info.iface = NULL;
  info.protocols = &this->protocols[0];

  if (!sslCertFile.empty() && !sslPrivateKeyFile.empty())
  {
    // Fail if the certificate file cannot be opened.
    if (!gz::common::exists(sslCertFile))
    {
      ignerr << "SSL certificate file[" << sslCertFile
        << "] does not exist. Quitting.\n";
      return false;
    }

    // Fail if the private key file cannot be opened.
    if (!gz::common::exists(sslPrivateKeyFile))
    {
      ignerr << "SSL private key file[" << sslPrivateKeyFile
        << "] does not exist. Quitting.\n";
      return false;
    }

    // Store SSL configuration.
    info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

    info.ssl_cert_filepath = sslCertFile.c_str();
    info.ssl_private_key_filepath = sslPrivateKeyFile.c_str();
  }
  else if (sslCertFile.empty() || sslPrivateKeyFile.empty())
  {
    ignwarn << "Partial SSL configuration specified. Please specify: "
    << "\t<ssl>\n"
    << "\t  <cert_file>PATH_TO_CERT_FILE</cert_file>\n"
    << "\t  <private_key_file>PATH_TO_KEY_FILE</private_key_file>\n"
    << "\t</ssl>.\n"
    << "Continuing without SSL.\n";
  }

  // keep alive time of  60 seconds
  info.ka_time = 60;
  // 10 probes after keep alive time
  info.ka_probes = 10;
  // 10s interval for sending probes
  info.ka_interval = 10;

  this->context = lws_create_context(&info);
  if( !this->context )
    ignerr << "Unable to create websocket server\n";

  this->run = true;
  this->thread = new std::thread(std::bind(&WebsocketServer::Run, this));

  return true;
}

//////////////////////////////////////////////////
void WebsocketServer::QueueMessage(Connection *_connection,
    const char *_data, const size_t _size)
{
  if (_connection)
  {
    std::unique_ptr<char> buf(new char[LWS_PRE + _size]);

    // Copy the message.
    memcpy(buf.get() + LWS_PRE, _data, _size);

    std::lock_guard<std::mutex> lock(_connection->mutex);
    _connection->buffer.push_back(std::move(buf));
    _connection->len.push_back(_size);

    std::scoped_lock<std::mutex> runLock(this->runMutex);
    this->messageCount++;
    this->runConditionVariable.notify_all();
  }
  else
  {
    ignerr << "Null pointer to a conection. This should not happen.\n";
  }
}

//////////////////////////////////////////////////
void WebsocketServer::Run()
{
  using namespace std::chrono_literals;

  while (this->run)
  {
    // The second parameter is a timeout that is no longer used by
    // libwebsockets.
    lws_service(this->context, 0);

    // Wait for (1/60) seconds or an event.
    std::unique_lock<std::mutex> lock(this->runMutex);
    this->runConditionVariable.wait_for(lock,
        0.0166s, [&]{return !this->run || this->messageCount > 0;});
  }
}

//////////////////////////////////////////////////
void WebsocketServer::OnConnect(int _socketId)
{
  std::unique_ptr<Connection> c(new Connection);
  c->creationTime = IGN_SYSTEM_TIME();

  // No authorization key means the server is publically accessible
  c->authorized = this->authorizationKey.empty() &&
                  this->adminAuthorizationKey.empty();
  this->connections[_socketId] = std::move(c);
}

//////////////////////////////////////////////////
void WebsocketServer::OnDisconnect(int _socketId)
{
  std::lock_guard<std::mutex> mainLock(this->subscriptionMutex);
  // Skip invalid sockets
  if (this->connections.find(_socketId) == this->connections.end())
    return;

  this->connections.erase(_socketId);

  // Somewhat slow operation.
  for (std::map<std::string, std::set<int>>::iterator iter =
       this->topicConnections.begin(); iter != this->topicConnections.end();
       ++iter)
  {
    iter->second.erase(_socketId);

    // Unsubscribe from the Ignition Transport topic if there are no more
    // websocket connections.
    if (iter->second.empty())
      this->node.Unsubscribe(iter->first);
  }
}

//////////////////////////////////////////////////
void WebsocketServer::OnMessage(int _socketId, const std::string &_msg)
{
  // Skip invalid sockets
  if (this->connections.find(_socketId) == this->connections.end())
    return;

  // Frame: operation,topic,type,payload
  std::vector<std::string> frameParts = common::split(_msg, ",");

  // Check for a valid frame.
  if (frameParts.size() != 4 &&
      // Count the number of commas to handle a frame like "sub,,,"
      std::count(_msg.begin(), _msg.end(), ',') != 3)
  {
    ignerr << "Received an invalid frame with " << frameParts.size()
      << "components when 4 is expected.\n";
    return;
  }

  // Check authorization
  if (frameParts[0] == "auth" &&
      (!this->authorizationKey.empty() || !this->adminAuthorizationKey.empty()))
  {
    std::string key = "";
    if (frameParts.size() > 1)
      key = frameParts.back();

    // Only check if the key is not empty.
    if (!key.empty())
    {
      this->connections[_socketId]->authorized =
        key == this->authorizationKey ||
        key == this->adminAuthorizationKey;
    }

    igndbg << "Authorization request received on socket[" << _socketId << "]. "
      << "Authorized[" << this->connections[_socketId]->authorized << "]\n";

    std::string result =
      this->connections[_socketId]->authorized ? "authorized": "invalid";

    this->QueueMessage(this->connections[_socketId].get(),
        result.c_str(), result.size());
  }

  if (!this->connections[_socketId]->authorized)
  {
    igndbg << "Unauthorized request received on socket[" << _socketId << "]\n";
    return;
  }

  // Handle the case where the client requests the message definitions.
  if (frameParts[0] == "protos")
  {
    igndbg << "Protos request received\n";

    std::string allProtos = "syntax = \"proto3\";\n";
    allProtos += "package ignition.msgs;\n";

    std::vector<std::string> types;
    gz::msgs::Factory::Types(types);

    // Get all the messages, and build a single proto to send to the client.
    for (auto const &type : types)
    {
      auto msg = gz::msgs::Factory::New(type);
      if (msg)
      {
        auto descriptor = msg->GetDescriptor();
        if (descriptor)
          allProtos += descriptor->DebugString();
        else
        {
          ignerr << "Failed to get the descriptor for message["
            << type << "]\n";
        }
      }
      else
      {
        ignerr << "Failed to build message[" << type << "].\n";
      }
    }

    this->QueueMessage(this->connections[_socketId].get(),
        allProtos.c_str(), allProtos.length());
  }
  else if (frameParts[0] == "topics")
  {
    igndbg << "Topic list request recieved\n";
    gz::msgs::StringMsg_V msg;

    std::vector<std::string> topics;

    // Get the list of topics
    this->node.TopicList(topics);

    // Store the topics in a message and serialize the message.
    for (const std::string &topic : topics)
      msg.add_data(topic);

    std::string data = BUILD_MSG(this->operations[PUBLISH], frameParts[0],
        std::string("ignition.msgs.StringMsg_V"), msg.SerializeAsString());

    // Queue the message for delivery.
    this->QueueMessage(this->connections[_socketId].get(),
        data.c_str(), data.length());
  }
  else if (frameParts[0] == "worlds")
  {
    igndbg << "World info request recieved\n";
    gz::msgs::Empty req;
    req.set_unused(true);

    gz::msgs::StringMsg_V rep;
    bool result;
    unsigned int timeout = 2000;

    bool executed = this->node.Request("/gazebo/worlds",
        req, timeout, rep, result);

    std::string data = BUILD_MSG(this->operations[PUBLISH], frameParts[0],
        std::string("ignition.msgs.StringMsg_V"), rep.SerializeAsString());

    // Queue the message for delivery.
    this->QueueMessage(this->connections[_socketId].get(),
        data.c_str(), data.length());
  }
  else if (frameParts[0] == "scene")
  {
    igndbg << "Scene info request recieved for world["
      << frameParts[1] << "]\n";
    gz::msgs::Empty req;
    req.set_unused(true);

    gz::msgs::Scene rep;
    bool result;
    unsigned int timeout = 2000;

    std::string serviceName = std::string("/world/") + frameParts[1] +
      "/scene/info";

    bool executed = this->node.Request(serviceName, req, timeout, rep, result);
    if (!executed || !result)
    {
      ignerr << "Failed to get the scene information for " << frameParts[1]
        << " world.\n";
    }

    std::string data = BUILD_MSG(this->operations[PUBLISH], frameParts[0],
        std::string("ignition.msgs.Scene"), rep.SerializeAsString());

    // Queue the message for delivery.
    this->QueueMessage(this->connections[_socketId].get(),
        data.c_str(), data.length());
  }
  else if (frameParts[0] == "sub")
  {
    // Store the relation of socketId to subscribed topic.
    this->topicConnections[frameParts[1]].insert(_socketId);
    this->topicTimestamps[frameParts[1]] =
      std::chrono::steady_clock::now() - this->publishPeriod;

    igndbg << "Subscribe request to topic[" << frameParts[1] << "]\n";
    this->node.SubscribeRaw(frameParts[1],
        std::bind(&WebsocketServer::OnWebsocketSubscribedMessage,
          this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
  }
}

//////////////////////////////////////////////////
void WebsocketServer::OnWebsocketSubscribedMessage(
    const char *_data, const size_t _size,
    const gz::transport::MessageInfo &_info)
{
  std::map<std::string, std::set<int>>::const_iterator iter =
    this->topicConnections.find(_info.Topic());

  if (iter != this->topicConnections.end())
  {
    std::lock_guard<std::mutex> mainLock(this->subscriptionMutex);
    std::chrono::time_point<std::chrono::steady_clock> systemTime =
      std::chrono::steady_clock::now();

    std::chrono::nanoseconds timeDelta =
      systemTime - this->topicTimestamps[_info.Topic()];

    if (timeDelta > this->publishPeriod)
    {
      // Get the header, or build a new header if it doesn't exist.
      auto header = this->publishHeaders.find(_info.Topic());
      if (header == this->publishHeaders.end())
      {
        this->publishHeaders[_info.Topic()] = BUILD_HEADER(
          this->operations[PUBLISH], _info.Topic(), _info.Type());
        header = this->publishHeaders.find(_info.Topic());
      }

      // Store the last time this topic was published.
      this->topicTimestamps[_info.Topic()] = systemTime;

      // Construct the final message.
      std::string msg = header->second + std::string(_data, _size);

      // Send the message
      for (const int &socketId : iter->second)
      {
        if (this->connections.find(socketId) != this->connections.end())
        {
          this->QueueMessage(this->connections[socketId].get(),
              msg.c_str(), msg.length());
        }
      }
    }
  }
}
