// Auto-generated. Do not edit!

// (in-package ros_flappy_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CameraFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.width = null;
      this.height = null;
      this.step = null;
      this.data_length = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = '';
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = 0;
      }
      if (initObj.hasOwnProperty('data_length')) {
        this.data_length = initObj.data_length
      }
      else {
        this.data_length = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraFrame
    // Serialize message field [header]
    bufferOffset = _serializer.string(obj.header, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.uint32(obj.height, buffer, bufferOffset);
    // Serialize message field [step]
    bufferOffset = _serializer.uint32(obj.step, buffer, bufferOffset);
    // Serialize message field [data_length]
    bufferOffset = _serializer.uint32(obj.data_length, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint8(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraFrame
    let len;
    let data = new CameraFrame(null);
    // Deserialize message field [header]
    data.header = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [step]
    data.step = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data_length]
    data.data_length = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.header);
    length += object.data.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_flappy_sim/CameraFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c052b18bb3b86a43759aefbf998d6f5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string header
    uint32 width
    uint32 height
    uint32 step
    uint32 data_length
    uint8[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraFrame(null);
    if (msg.header !== undefined) {
      resolved.header = msg.header;
    }
    else {
      resolved.header = ''
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = 0
    }

    if (msg.data_length !== undefined) {
      resolved.data_length = msg.data_length;
    }
    else {
      resolved.data_length = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = CameraFrame;
