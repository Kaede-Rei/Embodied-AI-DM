// Auto-generated. Do not edit!

// (in-package dm_arm_msgs_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class dm_arm_cmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.param1 = null;
      this.param2 = null;
      this.param3 = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('param1')) {
        this.param1 = initObj.param1
      }
      else {
        this.param1 = '';
      }
      if (initObj.hasOwnProperty('param2')) {
        this.param2 = initObj.param2
      }
      else {
        this.param2 = '';
      }
      if (initObj.hasOwnProperty('param3')) {
        this.param3 = initObj.param3
      }
      else {
        this.param3 = '';
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dm_arm_cmdRequest
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [param1]
    bufferOffset = _serializer.string(obj.param1, buffer, bufferOffset);
    // Serialize message field [param2]
    bufferOffset = _serializer.string(obj.param2, buffer, bufferOffset);
    // Serialize message field [param3]
    bufferOffset = _serializer.string(obj.param3, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dm_arm_cmdRequest
    let len;
    let data = new dm_arm_cmdRequest(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [param1]
    data.param1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [param2]
    data.param2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [param3]
    data.param3 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.command);
    length += _getByteLength(object.param1);
    length += _getByteLength(object.param2);
    length += _getByteLength(object.param3);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dm_arm_msgs_srvs/dm_arm_cmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e14b35af0de9d5af1d4dbee6b2e8766b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 请求部分
    string command      # 命令
    string param1       # 参数1: 备用参数
    string param2       # 参数2: 备用参数
    string param3       # 参数3: 备用参数
    float64 x           # 目标位置x坐标
    float64 y           # 目标位置y坐标
    float64 z           # 目标位置z坐标
    float64 roll        # 目标姿态roll角
    float64 pitch       # 目标姿态pitch角
    float64 yaw         # 目标姿态yaw角
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dm_arm_cmdRequest(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.param1 !== undefined) {
      resolved.param1 = msg.param1;
    }
    else {
      resolved.param1 = ''
    }

    if (msg.param2 !== undefined) {
      resolved.param2 = msg.param2;
    }
    else {
      resolved.param2 = ''
    }

    if (msg.param3 !== undefined) {
      resolved.param3 = msg.param3;
    }
    else {
      resolved.param3 = ''
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    return resolved;
    }
};

class dm_arm_cmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.cur_x = null;
      this.cur_y = null;
      this.cur_z = null;
      this.cur_roll = null;
      this.cur_pitch = null;
      this.cur_yaw = null;
      this.cur_joint = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('cur_x')) {
        this.cur_x = initObj.cur_x
      }
      else {
        this.cur_x = 0.0;
      }
      if (initObj.hasOwnProperty('cur_y')) {
        this.cur_y = initObj.cur_y
      }
      else {
        this.cur_y = 0.0;
      }
      if (initObj.hasOwnProperty('cur_z')) {
        this.cur_z = initObj.cur_z
      }
      else {
        this.cur_z = 0.0;
      }
      if (initObj.hasOwnProperty('cur_roll')) {
        this.cur_roll = initObj.cur_roll
      }
      else {
        this.cur_roll = 0.0;
      }
      if (initObj.hasOwnProperty('cur_pitch')) {
        this.cur_pitch = initObj.cur_pitch
      }
      else {
        this.cur_pitch = 0.0;
      }
      if (initObj.hasOwnProperty('cur_yaw')) {
        this.cur_yaw = initObj.cur_yaw
      }
      else {
        this.cur_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('cur_joint')) {
        this.cur_joint = initObj.cur_joint
      }
      else {
        this.cur_joint = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dm_arm_cmdResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [cur_x]
    bufferOffset = _serializer.float64(obj.cur_x, buffer, bufferOffset);
    // Serialize message field [cur_y]
    bufferOffset = _serializer.float64(obj.cur_y, buffer, bufferOffset);
    // Serialize message field [cur_z]
    bufferOffset = _serializer.float64(obj.cur_z, buffer, bufferOffset);
    // Serialize message field [cur_roll]
    bufferOffset = _serializer.float64(obj.cur_roll, buffer, bufferOffset);
    // Serialize message field [cur_pitch]
    bufferOffset = _serializer.float64(obj.cur_pitch, buffer, bufferOffset);
    // Serialize message field [cur_yaw]
    bufferOffset = _serializer.float64(obj.cur_yaw, buffer, bufferOffset);
    // Serialize message field [cur_joint]
    bufferOffset = _arraySerializer.float64(obj.cur_joint, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dm_arm_cmdResponse
    let len;
    let data = new dm_arm_cmdResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cur_x]
    data.cur_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_y]
    data.cur_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_z]
    data.cur_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_roll]
    data.cur_roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_pitch]
    data.cur_pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_yaw]
    data.cur_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cur_joint]
    data.cur_joint = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += 8 * object.cur_joint.length;
    return length + 57;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dm_arm_msgs_srvs/dm_arm_cmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e1b53476df714dd5d04f5cf9fe8e844';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 响应部分
    bool success        # 执行是否成功
    string message      # 返回消息
    float64 cur_x       # 当前x坐标
    float64 cur_y       # 当前y坐标
    float64 cur_z       # 当前z坐标
    float64 cur_roll    # 当前roll角
    float64 cur_pitch   # 当前pitch角
    float64 cur_yaw     # 当前yaw角
    float64[] cur_joint # 当前各关节位置数组 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dm_arm_cmdResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.cur_x !== undefined) {
      resolved.cur_x = msg.cur_x;
    }
    else {
      resolved.cur_x = 0.0
    }

    if (msg.cur_y !== undefined) {
      resolved.cur_y = msg.cur_y;
    }
    else {
      resolved.cur_y = 0.0
    }

    if (msg.cur_z !== undefined) {
      resolved.cur_z = msg.cur_z;
    }
    else {
      resolved.cur_z = 0.0
    }

    if (msg.cur_roll !== undefined) {
      resolved.cur_roll = msg.cur_roll;
    }
    else {
      resolved.cur_roll = 0.0
    }

    if (msg.cur_pitch !== undefined) {
      resolved.cur_pitch = msg.cur_pitch;
    }
    else {
      resolved.cur_pitch = 0.0
    }

    if (msg.cur_yaw !== undefined) {
      resolved.cur_yaw = msg.cur_yaw;
    }
    else {
      resolved.cur_yaw = 0.0
    }

    if (msg.cur_joint !== undefined) {
      resolved.cur_joint = msg.cur_joint;
    }
    else {
      resolved.cur_joint = []
    }

    return resolved;
    }
};

module.exports = {
  Request: dm_arm_cmdRequest,
  Response: dm_arm_cmdResponse,
  md5sum() { return 'ab876838156246ec2905cd94b5756b0a'; },
  datatype() { return 'dm_arm_msgs_srvs/dm_arm_cmd'; }
};
