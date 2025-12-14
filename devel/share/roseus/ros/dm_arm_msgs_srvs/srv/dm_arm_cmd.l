;; Auto-generated. Do not edit!


(when (boundp 'dm_arm_msgs_srvs::dm_arm_cmd)
  (if (not (find-package "DM_ARM_MSGS_SRVS"))
    (make-package "DM_ARM_MSGS_SRVS"))
  (shadow 'dm_arm_cmd (find-package "DM_ARM_MSGS_SRVS")))
(unless (find-package "DM_ARM_MSGS_SRVS::DM_ARM_CMD")
  (make-package "DM_ARM_MSGS_SRVS::DM_ARM_CMD"))
(unless (find-package "DM_ARM_MSGS_SRVS::DM_ARM_CMDREQUEST")
  (make-package "DM_ARM_MSGS_SRVS::DM_ARM_CMDREQUEST"))
(unless (find-package "DM_ARM_MSGS_SRVS::DM_ARM_CMDRESPONSE")
  (make-package "DM_ARM_MSGS_SRVS::DM_ARM_CMDRESPONSE"))

(in-package "ROS")





(defclass dm_arm_msgs_srvs::dm_arm_cmdRequest
  :super ros::object
  :slots (_command _param1 _param2 _param3 _x _y _z _roll _pitch _yaw ))

(defmethod dm_arm_msgs_srvs::dm_arm_cmdRequest
  (:init
   (&key
    ((:command __command) "")
    ((:param1 __param1) "")
    ((:param2 __param2) "")
    ((:param3 __param3) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _param1 (string __param1))
   (setq _param2 (string __param2))
   (setq _param3 (string __param3))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:param1
   (&optional __param1)
   (if __param1 (setq _param1 __param1)) _param1)
  (:param2
   (&optional __param2)
   (if __param2 (setq _param2 __param2)) _param2)
  (:param3
   (&optional __param3)
   (if __param3 (setq _param3 __param3)) _param3)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _param1
    4 (length _param1)
    ;; string _param2
    4 (length _param2)
    ;; string _param3
    4 (length _param3)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _param1
       (write-long (length _param1) s) (princ _param1 s)
     ;; string _param2
       (write-long (length _param2) s) (princ _param2 s)
     ;; string _param3
       (write-long (length _param3) s) (princ _param3 s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _param3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _param3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dm_arm_msgs_srvs::dm_arm_cmdResponse
  :super ros::object
  :slots (_success _message _cur_x _cur_y _cur_z _cur_roll _cur_pitch _cur_yaw _cur_joint ))

(defmethod dm_arm_msgs_srvs::dm_arm_cmdResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:cur_x __cur_x) 0.0)
    ((:cur_y __cur_y) 0.0)
    ((:cur_z __cur_z) 0.0)
    ((:cur_roll __cur_roll) 0.0)
    ((:cur_pitch __cur_pitch) 0.0)
    ((:cur_yaw __cur_yaw) 0.0)
    ((:cur_joint __cur_joint) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _cur_x (float __cur_x))
   (setq _cur_y (float __cur_y))
   (setq _cur_z (float __cur_z))
   (setq _cur_roll (float __cur_roll))
   (setq _cur_pitch (float __cur_pitch))
   (setq _cur_yaw (float __cur_yaw))
   (setq _cur_joint __cur_joint)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:cur_x
   (&optional __cur_x)
   (if __cur_x (setq _cur_x __cur_x)) _cur_x)
  (:cur_y
   (&optional __cur_y)
   (if __cur_y (setq _cur_y __cur_y)) _cur_y)
  (:cur_z
   (&optional __cur_z)
   (if __cur_z (setq _cur_z __cur_z)) _cur_z)
  (:cur_roll
   (&optional __cur_roll)
   (if __cur_roll (setq _cur_roll __cur_roll)) _cur_roll)
  (:cur_pitch
   (&optional __cur_pitch)
   (if __cur_pitch (setq _cur_pitch __cur_pitch)) _cur_pitch)
  (:cur_yaw
   (&optional __cur_yaw)
   (if __cur_yaw (setq _cur_yaw __cur_yaw)) _cur_yaw)
  (:cur_joint
   (&optional __cur_joint)
   (if __cur_joint (setq _cur_joint __cur_joint)) _cur_joint)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; float64 _cur_x
    8
    ;; float64 _cur_y
    8
    ;; float64 _cur_z
    8
    ;; float64 _cur_roll
    8
    ;; float64 _cur_pitch
    8
    ;; float64 _cur_yaw
    8
    ;; float64[] _cur_joint
    (* 8    (length _cur_joint)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float64 _cur_x
       (sys::poke _cur_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cur_y
       (sys::poke _cur_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cur_z
       (sys::poke _cur_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cur_roll
       (sys::poke _cur_roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cur_pitch
       (sys::poke _cur_pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cur_yaw
       (sys::poke _cur_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _cur_joint
     (write-long (length _cur_joint) s)
     (dotimes (i (length _cur_joint))
       (sys::poke (elt _cur_joint i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _cur_x
     (setq _cur_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cur_y
     (setq _cur_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cur_z
     (setq _cur_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cur_roll
     (setq _cur_roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cur_pitch
     (setq _cur_pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cur_yaw
     (setq _cur_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _cur_joint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cur_joint (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cur_joint i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass dm_arm_msgs_srvs::dm_arm_cmd
  :super ros::object
  :slots ())

(setf (get dm_arm_msgs_srvs::dm_arm_cmd :md5sum-) "ab876838156246ec2905cd94b5756b0a")
(setf (get dm_arm_msgs_srvs::dm_arm_cmd :datatype-) "dm_arm_msgs_srvs/dm_arm_cmd")
(setf (get dm_arm_msgs_srvs::dm_arm_cmd :request) dm_arm_msgs_srvs::dm_arm_cmdRequest)
(setf (get dm_arm_msgs_srvs::dm_arm_cmd :response) dm_arm_msgs_srvs::dm_arm_cmdResponse)

(defmethod dm_arm_msgs_srvs::dm_arm_cmdRequest
  (:response () (instance dm_arm_msgs_srvs::dm_arm_cmdResponse :init)))

(setf (get dm_arm_msgs_srvs::dm_arm_cmdRequest :md5sum-) "ab876838156246ec2905cd94b5756b0a")
(setf (get dm_arm_msgs_srvs::dm_arm_cmdRequest :datatype-) "dm_arm_msgs_srvs/dm_arm_cmdRequest")
(setf (get dm_arm_msgs_srvs::dm_arm_cmdRequest :definition-)
      "# 请求部分
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
---
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
")

(setf (get dm_arm_msgs_srvs::dm_arm_cmdResponse :md5sum-) "ab876838156246ec2905cd94b5756b0a")
(setf (get dm_arm_msgs_srvs::dm_arm_cmdResponse :datatype-) "dm_arm_msgs_srvs/dm_arm_cmdResponse")
(setf (get dm_arm_msgs_srvs::dm_arm_cmdResponse :definition-)
      "# 请求部分
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
---
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
")



(provide :dm_arm_msgs_srvs/dm_arm_cmd "ab876838156246ec2905cd94b5756b0a")


