; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude neighbor_camera_data.msg.html

(cl:defclass <neighbor_camera_data> (roslisp-msg-protocol:ros-message)
  ((UAV_ID
    :reader UAV_ID
    :initarg :UAV_ID
    :type cl:fixnum
    :initform 0)
   (UAV_OBJ_X
    :reader UAV_OBJ_X
    :initarg :UAV_OBJ_X
    :type cl:fixnum
    :initform 0)
   (UAV_OBJ_Y
    :reader UAV_OBJ_Y
    :initarg :UAV_OBJ_Y
    :type cl:fixnum
    :initform 0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:integer
    :initform 0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:integer
    :initform 0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:fixnum
    :initform 0))
)

(cl:defclass neighbor_camera_data (<neighbor_camera_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <neighbor_camera_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'neighbor_camera_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<neighbor_camera_data> is deprecated: use camera-msg:neighbor_camera_data instead.")))

(cl:ensure-generic-function 'UAV_ID-val :lambda-list '(m))
(cl:defmethod UAV_ID-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:UAV_ID-val is deprecated.  Use camera-msg:UAV_ID instead.")
  (UAV_ID m))

(cl:ensure-generic-function 'UAV_OBJ_X-val :lambda-list '(m))
(cl:defmethod UAV_OBJ_X-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:UAV_OBJ_X-val is deprecated.  Use camera-msg:UAV_OBJ_X instead.")
  (UAV_OBJ_X m))

(cl:ensure-generic-function 'UAV_OBJ_Y-val :lambda-list '(m))
(cl:defmethod UAV_OBJ_Y-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:UAV_OBJ_Y-val is deprecated.  Use camera-msg:UAV_OBJ_Y instead.")
  (UAV_OBJ_Y m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:latitude-val is deprecated.  Use camera-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:longitude-val is deprecated.  Use camera-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <neighbor_camera_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:altitude-val is deprecated.  Use camera-msg:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <neighbor_camera_data>) ostream)
  "Serializes a message object of type '<neighbor_camera_data>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_OBJ_X)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'UAV_OBJ_X)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_OBJ_Y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'UAV_OBJ_Y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'latitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'latitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'longitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'longitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'longitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'longitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'altitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'altitude)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <neighbor_camera_data>) istream)
  "Deserializes a message object of type '<neighbor_camera_data>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_OBJ_X)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'UAV_OBJ_X)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'UAV_OBJ_Y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'UAV_OBJ_Y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'latitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'latitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'longitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'longitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'longitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'longitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'altitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'altitude)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<neighbor_camera_data>)))
  "Returns string type for a message object of type '<neighbor_camera_data>"
  "camera/neighbor_camera_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'neighbor_camera_data)))
  "Returns string type for a message object of type 'neighbor_camera_data"
  "camera/neighbor_camera_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<neighbor_camera_data>)))
  "Returns md5sum for a message object of type '<neighbor_camera_data>"
  "cf9983e93e6a13e8480601768a9c54d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'neighbor_camera_data)))
  "Returns md5sum for a message object of type 'neighbor_camera_data"
  "cf9983e93e6a13e8480601768a9c54d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<neighbor_camera_data>)))
  "Returns full string definition for message of type '<neighbor_camera_data>"
  (cl:format cl:nil "uint8   UAV_ID~%#物体相对位置估计~%uint16 UAV_OBJ_X~%uint16 UAV_OBJ_Y~%#无人机信息~%uint32 latitude~%uint32 longitude~%uint16 altitude~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'neighbor_camera_data)))
  "Returns full string definition for message of type 'neighbor_camera_data"
  (cl:format cl:nil "uint8   UAV_ID~%#物体相对位置估计~%uint16 UAV_OBJ_X~%uint16 UAV_OBJ_Y~%#无人机信息~%uint32 latitude~%uint32 longitude~%uint16 altitude~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <neighbor_camera_data>))
  (cl:+ 0
     1
     2
     2
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <neighbor_camera_data>))
  "Converts a ROS message object to a list"
  (cl:list 'neighbor_camera_data
    (cl:cons ':UAV_ID (UAV_ID msg))
    (cl:cons ':UAV_OBJ_X (UAV_OBJ_X msg))
    (cl:cons ':UAV_OBJ_Y (UAV_OBJ_Y msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
))
