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

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs.hh>

#include "WebsocketServer.hh"

using namespace ignition::launch;

int rootCallback(struct lws *_wsi,
                 enum lws_callback_reasons _reason,
                 void * /*user*/,
                 void * _in,
                 size_t /*len*/)
{
  WebsocketServer *self = nullptr;

  // Get the protocol definition for this callback
  lws_protocols *protocol = const_cast<lws_protocols*>(
      lws_get_protocol(_wsi));

  // It's possible that the protocol is null.
  if (protocol)
    self = static_cast<WebsocketServer*>(protocol->user);

  // We require the self pointer, and ignore the cases when this function is
  // called without a self pointer.
  if (!self)
    return 0;

  int fd = lws_get_socket_fd(_wsi);

  // std::lock_guard<std::mutex> mainLock(self->mutex);
  switch (_reason)
  {
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

    // Publish outboud messages
    case LWS_CALLBACK_SERVER_WRITEABLE:
      {
        std::lock_guard<std::mutex> lock(self->connections[fd]->mutex);
        //while (!self->connections[fd]->buffer.empty())
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
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
WebsocketServer::~WebsocketServer()
{
  if (this->thread && this->run)
  {
    this->run = false;
    this->thread->join();
  }
  this->thread = nullptr;

  if (this->context)
    lws_context_destroy(this->context);
}

/////////////////////////////////////////////////
bool WebsocketServer::Load(const tinyxml2::XMLElement * /*_elem*/)
{
  // All of the protocols handled by this websocket server.
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

  int port = 9002;

  // We will handle logging
  lws_set_log_level( 0, lwsl_emit_syslog);

  struct lws_context_creation_info info;
  memset( &info, 0, sizeof info );
  info.port = port;
  info.iface = NULL;
  info.protocols = &this->protocols[0];
  // We are not using SSL right now
  info.ssl_cert_filepath        = NULL;
  info.ssl_private_key_filepath = NULL;

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
  }
  else
  {
    ignerr << "Null pointer to a conection. This should not happen.\n";
  }
}

//////////////////////////////////////////////////
void WebsocketServer::Run()
{
  uint64_t timeout = 50;
  while (this->run)
    lws_service(this->context, timeout);
}

//////////////////////////////////////////////////
void WebsocketServer::OnConnect(int _socketId)
{
  std::unique_ptr<Connection> c(new Connection);
  c->creationTime = IGN_SYSTEM_TIME();
  this->connections[_socketId] = std::move(c);
}

//////////////////////////////////////////////////
void WebsocketServer::OnDisconnect(int _socketId)
{
  this->connections.erase(_socketId);

  // Somewhat slow operation.
  for (std::map<std::string, std::set<int>>::iterator iter =
       this->topicConnections.begin(); iter != this->topicConnections.end();
       ++iter)
  {
    iter->second.erase(_socketId);
  }
}

//////////////////////////////////////////////////
void WebsocketServer::OnMessage(int _socketId, const std::string &_msg)
{
  ignition::msgs::WebRequest requestMsg;
  requestMsg.ParseFromString(_msg);

  if (requestMsg.operation() == "list")
  {
    igndbg << "Topic list request recieved\n";
    ignition::msgs::Packet msg;

    std::vector<std::string> topics;

    // Get the list of topics
    this->node.TopicList(topics);

    // Store the topics in a message and serialize the message.
    for (const std::string &topic : topics)
      msg.mutable_string_msg_v()->add_data(topic);

    std::string data = msg.SerializeAsString();

    // Queue the message for delivery.
    this->QueueMessage(this->connections[_socketId].get(),
        data.c_str(), data.length());
  }
  else if (requestMsg.operation() == "subscribe")
  {
    // Store the relation of socketId to subscribed topic.
    this->topicConnections[requestMsg.topic()].insert(_socketId);

    igndbg << "Subscribe request to topic[" << requestMsg.topic() << "]\n";
    this->node.SubscribeRaw(requestMsg.topic(),
        std::bind(&WebsocketServer::OnWebsocketSubscribedMessage,
          this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
  }
}

//////////////////////////////////////////////////
void WebsocketServer::OnWebsocketSubscribedMessage(
    const char *_data, const size_t _size,
    const ignition::transport::MessageInfo &_info)
{
  std::map<std::string, std::set<int>>::const_iterator iter =
    this->topicConnections.find(_info.Topic());

  if (iter != this->topicConnections.end())
  {
    ignition::msgs::Packet msg;
    msg.set_topic(_info.Topic());
    msg.set_type(_info.Type());

    if (_info.Type() == "ignition.msgs.CmdVel2D")
      msg.mutable_cmd_vel2d()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.Image")
      msg.mutable_image()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.StringMsg_V")
      msg.mutable_string_msg_v()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.WebRequest")
      msg.mutable_web_request()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.Pose")
      msg.mutable_pose()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.Pose_V")
      msg.mutable_pose_v()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.Time")
      msg.mutable_time()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.Clock")
      msg.mutable_clock()->ParseFromArray(_data, _size);
    else if (_info.Type() == "ignition.msgs.WorldStatistics")
      msg.mutable_world_stats()->ParseFromArray(_data, _size);

    std::string data = msg.SerializeAsString();
    for (const int &socketId : iter->second)
    {
      this->QueueMessage(this->connections[socketId].get(),
          data.c_str(), data.length());
    }
  }
}
