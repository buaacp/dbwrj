// Auto-generated. Do not edit!

// (in-package camera.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class pos_message {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.relate_E = null;
      this.relate_N = null;
      this.tar_lat = null;
      this.tar_lon = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('relate_E')) {
        this.relate_E = initObj.relate_E
      }
      else {
        this.relate_E = 0.0;
      }
      if (initObj.hasOwnProperty('relate_N')) {
        this.relate_N = initObj.relate_N
      }
      else {
        this.relate_N = 0.0;
      }
      if (initObj.hasOwnProperty('tar_lat')) {
        this.tar_lat = initObj.tar_lat
      }
      else {
        this.tar_lat = 0.0;
      }
      if (initObj.hasOwnProperty('tar_lon')) {
        this.tar_lon = initObj.tar_lon
      }
      else {
        this.tar_lon = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pos_message
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [relate_E]
    bufferOffset = _serializer.float32(obj.relate_E, buffer, bufferOffset);
    // Serialize message field [relate_N]
    bufferOffset = _serializer.float32(obj.relate_N, buffer, bufferOffset);
    // Serialize message field [tar_lat]
    bufferOffset = _serializer.float32(obj.tar_lat, buffer, bufferOffset);
    // Serialize message field [tar_lon]
    bufferOffset = _serializer.float32(obj.tar_lon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pos_message
    let len;
    let data = new pos_message(null);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [relate_E]
    data.relate_E = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [relate_N]
    data.relate_N = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tar_lat]
    data.tar_lat = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tar_lon]
    data.tar_lon = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera/pos_message';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d9e4de796d3278b9e92378ad269bd1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time time
    float32 relate_E #正东方向差距 单位 米
    float32 relate_N #正北方向差距 单位 米
    float32 tar_lat
    float32 tar_lon
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pos_message(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = {secs: 0, nsecs: 0}
    }

    if (msg.relate_E !== undefined) {
      resolved.relate_E = msg.relate_E;
    }
    else {
      resolved.relate_E = 0.0
    }

    if (msg.relate_N !== undefined) {
      resolved.relate_N = msg.relate_N;
    }
    else {
      resolved.relate_N = 0.0
    }

    if (msg.tar_lat !== undefined) {
      resolved.tar_lat = msg.tar_lat;
    }
    else {
      resolved.tar_lat = 0.0
    }

    if (msg.tar_lon !== undefined) {
      resolved.tar_lon = msg.tar_lon;
    }
    else {
      resolved.tar_lon = 0.0
    }

    return resolved;
    }
};

module.exports = pos_message;
