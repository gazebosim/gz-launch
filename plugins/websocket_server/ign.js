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

/// \brief The main interface to the Ignition websocket server and 
/// data on Ignition Transport.
function Ignition(options) {
  options = options || {};

  this.socket = null;
  this.topics = [];
  this.isConnected = false;

  // Start with a null root protobuf object. This object will be 
  // created when we get the set of protobuf definitions from the server.
  this.root = null;

  if (options.url) {
    this.connect(options.url);
  }
}
Ignition.prototype.__proto__ = EventEmitter2.prototype;

/// \brief Connect to the specified WebSocket.
/// \param url - WebSocket URL for Ignition HTTPServer
Ignition.prototype.connect = function(url) {
  var that = this;

  /// \brief Emits a 'connection' event on WebSocket connection.
  /// \param event - the argument to emit with the event.
  function onOpen(event) {
    that.socket.send("message_definitions");

  }

  /// \brief Emits a 'close' event on WebSocket disconnection.
  /// \param event - the argument to emit with the event.
  function onClose(event) {
    that.isConnected = false;
    that.emit('close', event);
  }

  /// \brief Emits an 'error' event whenever there was an error.
  /// \param event - the argument to emit with the event.
  function onError(event) {
    that.emit('error', event);
  }

  /// \brief Parses message responses from ignition and sends to the
  /// appropriate topic.
  // \param message - the JSON message from the Ignition
  // httpserver.
  function onMessage(_message) {

    if (that.root === undefined || that.root === null) {
      // Read the Blob as an array buffer
      var f = new FileReader();

      f.onloadend = function(event) {
        // This is the proto message data
        var contents = event.target.result;
        that.root = protobuf.parse(contents, {keepCase: true}).root;
        that.isConnected = true;
        that.emit('connection', event);

        // Request the list of topics on start.
        // \todo: Check that the "WebRequest message exists.
        var msgType = that.root.lookupType("WebRequest");
        var webRequestMsg = msgType.encode({
          operation: "topic_list",
        }).finish();
        that.socket.send(webRequestMsg);
      };

      // Read the blob data as an array buffer.
      f.readAsText(_message.data);
      return;
    }

    // \todo: Check that packetMsgType is valid
    var packetMsgType = that.root.lookup("ignition.msgs.Packet");

    // Read the Blob as an array buffer
    var f = new FileReader();
    f.onloadend = function(event) {
      // This is the proto message data
      var contents = event.target.result;

      // \todo: Check for error
      var error = event.target.error;

      // Get the decoded packet
      var packetMsg = packetMsgType.decode(new Uint8Array(contents));

      // Get the "oneof" message
      var which = packetMsg.content;
      var oneOfMsg = which !== null ? packetMsg[which] : null;

      // console.log("Actual topic: ", packetMsg.topic);
      // console.log("Actual type: ", packetMsg.type);
      // console.log("Actual Which: ", which);
      // console.log("Actual Msg: ", oneOfMsg);

      if (packetMsg.topic === "/topic_list") {
      // The "/topic_list" topic is a special case
        that.topics = oneOfMsg.data;
      }
      // Otherwise emit the message.
      else
      {
        // This will pass along the message on the appropriate topic.
        that.emit(packetMsg.topic, oneOfMsg);
      }
    }
    // Read the blob data as an array buffer.
    f.readAsArrayBuffer(_message.data);
  }

  this.socket = new WebSocket(url);
  this.socket.onopen = onOpen;
  this.socket.onclose = onClose;
  this.socket.onerror = onError;
  this.socket.onmessage = onMessage;
};

/// \brief Send a message to the websocket server
/// \param[in] _msg Message to send
Ignition.prototype.sendMsg = function(_msg) {
  var that = this;

  var emitter = function(msg){
    that.socket.send(msg);
  };

  // Wait for a connection before sending the message.
  if (!this.isConnected) {
    that.on('connection', function() {
      emitter(_msg);
    });
  } else {
    emitter(_msg);
  }
};

/// \brief Interface to Ignition Transport topics.
function Topic(options) {
  options = options || {};
  this.ign = options.ign; 
  this.name = options.name;
  this.messageType = options.messageType;
  this.isAdvertised = false;

  // Subscribe immediately if the callback is specified.
  if (options.callback) {
    this.subscribe(options.callback);
  }
}
Topic.prototype.__proto__ = EventEmitter2.prototype

// \brief Every time a message is published for the given topic, the callback
// will be called with the message object.
// \param[in] callback - function with the following params:
//   * message - the published message
Topic.prototype.subscribe = function(_callback) {
  var that = this;

  var emitter = function(_cb) {
    // Register the callback with the topic name
    that.ign.on(that.name, _cb);

    // \todo: Check that requesttMsgType is valid
    var requestMsgType = that.ign.root.lookup("ignition.msgs.WebRequest");

    // Create the subscription request message
    var webRequestMsg = requestMsgType.encode({
      operation: "subscribe",
      topic: that.name,
    }).finish();

    // Send the subscription message over the websocket.
    that.ign.sendMsg(webRequestMsg);
  }

  if (!this.ign.isConnected) {
    this.ign.on('connection', function() {
      emitter(_callback);
    });
  } else {
    emiiter(_callback);
  }
};
