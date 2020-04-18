var proto = "syntax = \"proto3\";\
  package ignition.msgs;\
  message Time {\
    int64 sec = 1;\
    int32 nsec = 2;\
  }\
  message Clock\
  {\
    Header header = 1;\
    Time system = 2;\
    Time real   = 3;\
    Time sim    = 4;\
  }\
  message Header {\
    message Map {\
      string key = 1;\
      repeated string value = 2;\
    }\
    Time stamp         = 1;\
    repeated Map data  = 2;\
  }\
  message WebRequest\
  {\
    Header header         = 1;\
    string operation      = 2;\
    string topic          = 3;\
    string msg_type       = 4;\
    string compression    = 5;\
    double hz             = 6;\
  }\
  message StringMsg_V\
  {\
    repeated string data = 2;\
  }\
  message CmdVel2D\
  {\
    Header header    = 1;\
    double velocity  = 2;\
    double theta     = 3;\
  }\
  enum PixelFormatType\
  {\
    UNKNOWN_PIXEL_FORMAT = 0;\
    L_INT8 = 1;\
    L_INT16 = 2;\
    RGB_INT8 = 3;\
    RGBA_INT8 = 4;\
    BGRA_INT8 = 5;\
    RGB_INT16 = 6;\
    RGB_INT32 = 7;\
    BGR_INT8 = 8;\
    BGR_INT16 = 9;\
    BGR_INT32 = 10;\
    R_FLOAT16 = 11;\
    RGB_FLOAT16 = 12;\
    R_FLOAT32 = 13;\
    RGB_FLOAT32 = 14;\
    BAYER_RGGB8 = 15;\
    BAYER_RGGR8 = 16;\
    BAYER_GBRG8 = 17;\
    BAYER_GRBG8 = 18;\
  }\
  \
  message Image\
  {\
    Header header        = 1;\
    uint32 width         = 2;\
    uint32 height        = 3;\
    uint32 pixel_format  = 4;\
    uint32 step          = 5;\
    bytes data          = 6;\
  }\
  message Vector3d\
  {\
    Header header = 1;\
    double x = 2;\
    double y = 3;\
    double z = 4;\
  }\
  message Pose\
  {\
    Header header          = 1;\
    string name            = 2;\
    uint32 id              = 3;\
    Vector3d position      = 4;\
    Quaternion orientation = 5;\
  }\
  message Quaternion\
  {\
    Header header = 1;\
    double x = 2;\
    double y = 3;\
    double z = 4;\
    double w = 5;\
  }\
  message Double_V\
  {\
    repeated double data = 1;\
  }\
  message Pose_V\
  {\
    Header header = 1;\
    repeated Pose pose = 2;\
  }\
  message Packet\
  {\
    string topic = 1;\
    string type  = 2;\
  \
    oneof content\
    {\
      CmdVel2D cmd_vel2d = 3;\
      Image image = 4;\
      StringMsg_V string_msg_v = 5;\
      WebRequest web_request = 6;\
      Pose pose = 7;\
      Double_V doublev = 8;\
      Pose_V pose_v = 9;\
      Time time = 10;\
      Clock clock = 11;\
    }\
  }";

function Transport(options) {
  options = options || {};

  this.socket = null;
  this.idCounter = 0;
  this.topics = [];
  this.isConnected = false;

  // Sets unlimited event listeners.
  this.setMaxListeners(0);

  // Create a protobuf root object
  // \todo: Get proto definitions from the server.
  this.root = protobuf.parse(proto, {keepCase: true}).root;
  // this.root = new protobuf.Root();

  if (options.url) {
    this.connect(options.url);
  }

  // This maps protobuf message type to .proto files.
  /*this.typeMap = {
      "ignition.msgs.Header": "msgs/header.proto",
      "ignition.msgs.Time": "msgs/time.proto",
      "ignition.msgs.StringMsg_V": "msgs/stringmsg_v.proto"
    };*/
}
Transport.prototype.__proto__ = EventEmitter2.prototype;

/// \brief Connect to the specified WebSocket.
/// \param url - WebSocket URL for Ignition HTTPServer
Transport.prototype.connect = function(url) {
  var that = this;

  /// \brief Emits a 'connection' event on WebSocket connection.
  /// \param event - the argument to emit with the event.
  function onOpen(event) {
    that.isConnected = true;
    that.emit('connection', event);

    // Request the list of topics on start.
    var msgType = that.root.lookupType("WebRequest");
    var webRequestMsg = msgType.encode({
      operation: "topic_list",
      compression: "none",
    }).finish();
    that.socket.send(webRequestMsg);
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

      // The "/topic_list" topic is a special case
      if (packetMsg.topic === "/topic_list")
      {
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

Transport.prototype.sendMsg = function(_msg) {
  var that = this;

  var emitter = function(msg){
    that.socket.send(msg);
  };

  if (!this.isConnected) {
    that.on('connection', function() {
      emitter(_msg);
    });
  } else {
    emitter(_msg);
  }
};

function Topic(options) {
  options = options || {};
  this.ign = options.ign; 
  this.name = options.name;
  this.messageType = options.messageType;
  this.isAdvertised = false;
  this.compression = options.compression || 'none';
  this.throttle_rate = options.throttle_rate || 0;

  // Check for valid compression types
  if (this.compression && this.compression !== 'png' &&
    this.compression !== 'cbor' && this.compression !== 'cbor-raw' &&
    this.compression !== 'none') {
    this.emit('warning', this.compression +
      ' compression is not supported. No compression will be used.');
    this.compression = 'none';
  }

  // Check if throttle rate is negative
  if (this.throttle_rate < 0) {
    this.emit('warning', this.throttle_rate + ' is not allowed. Set to 0');
    this.throttle_rate = 0;
  }

  var that = this;
  // this.sendMessage = this.ign.sendMsg;
  /*this.messageCallback = function(_msg) {
    that.emit('message', _msg);
  }*/
}
Topic.prototype.__proto__ = EventEmitter2.prototype

// \brief Every time a message is published for the given topic, the callback
// will be called with the message object.
// \param callback - function with the following params:
//   * message - the published message
Topic.prototype.subscribe = function(callback) {
  var that = this;

  /*if (typeof callback == 'function') {
    this.on('message', callback);
  }*/

  //this.ign.on(this.name, this.messageCallback);
  this.ign.on(this.name, callback);

  // this.ign.idCounter++;

  // \todo: Check that requesttMsgType is valid
  var requestMsgType = this.ign.root.lookup("ignition.msgs.WebRequest");

  // Create the request message
  var webRequestMsg = requestMsgType.encode({
    operation: "subscribe",
    topic: that.name,
    compression: "none",
  }).finish();

  // Send the message over the websocket.
  this.ign.sendMsg(webRequestMsg);
};
