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

class neighbor_camera_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.UAV_ID = null;
      this.UAV_OBJ_X = null;
      this.UAV_OBJ_Y = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
    }
    else {
      if (initObj.hasOwnProperty('UAV_ID')) {
        this.UAV_ID = initObj.UAV_ID
      }
      else {
        this.UAV_ID = 0;
      }
      if (initObj.hasOwnProperty('UAV_OBJ_X')) {
        this.UAV_OBJ_X = initObj.UAV_OBJ_X
      }
      else {
        this.UAV_OBJ_X = 0;
      }
      if (initObj.hasOwnProperty('UAV_OBJ_Y')) {
        this.UAV_OBJ_Y = initObj.UAV_OBJ_Y
      }
      else {
        this.UAV_OBJ_Y = 0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type neighbor_camera_data
    // Serialize message field [UAV_ID]
    bufferOffset = _serializer.uint8(obj.UAV_ID, buffer, bufferOffset);
    // Serialize message field [UAV_OBJ_X]
    bufferOffset = _serializer.uint16(obj.UAV_OBJ_X, buffer, bufferOffset);
    // Serialize message field [UAV_OBJ_Y]
    bufferOffset = _serializer.uint16(obj.UAV_OBJ_Y, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.uint32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.uint32(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.uint16(obj.altitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type neighbor_camera_data
    let len;
    let data = new neighbor_camera_data(null);
    // Deserialize message field [UAV_ID]
    data.UAV_ID = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [UAV_OBJ_X]
    data.UAV_OBJ_X = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [UAV_OBJ_Y]
    data.UAV_OBJ_Y = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera/neighbor_camera_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf9983e93e6a13e8480601768a9c54d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8   UAV_ID
    #物体相对位置估计
    uint16 UAV_OBJ_X
    uint16 UAV_OBJ_Y
    #无人机信息
    uint32 latitude
    uint32 longitude
    uint16 altitude
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new neighbor_camera_data(null);
    if (msg.UAV_ID !== undefined) {
      resolved.UAV_ID = msg.UAV_ID;
    }
    else {
      resolved.UAV_ID = 0
    }

    if (msg.UAV_OBJ_X !== undefined) {
      resolved.UAV_OBJ_X = msg.UAV_OBJ_X;
    }
    else {
      resolved.UAV_OBJ_X = 0
    }

    if (msg.UAV_OBJ_Y !== undefined) {
      resolved.UAV_OBJ_Y = msg.UAV_OBJ_Y;
    }
    else {
      resolved.UAV_OBJ_Y = 0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0
    }

    return resolved;
    }
};

module.exports = neighbor_camera_data;
