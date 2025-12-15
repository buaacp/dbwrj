; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude pos_message.msg.html

(cl:defclass <pos_message> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (relate_E
    :reader relate_E
    :initarg :relate_E
    :type cl:float
    :initform 0.0)
   (relate_N
    :reader relate_N
    :initarg :relate_N
    :type cl:float
    :initform 0.0)
   (tar_lat
    :reader tar_lat
    :initarg :tar_lat
    :type cl:float
    :initform 0.0)
   (tar_lon
    :reader tar_lon
    :initarg :tar_lon
    :type cl:float
    :initform 0.0))
)

(cl:defclass pos_message (<pos_message>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_message>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_message)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<pos_message> is deprecated: use camera-msg:pos_message instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <pos_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:time-val is deprecated.  Use camera-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'relate_E-val :lambda-list '(m))
(cl:defmethod relate_E-val ((m <pos_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:relate_E-val is deprecated.  Use camera-msg:relate_E instead.")
  (relate_E m))

(cl:ensure-generic-function 'relate_N-val :lambda-list '(m))
(cl:defmethod relate_N-val ((m <pos_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:relate_N-val is deprecated.  Use camera-msg:relate_N instead.")
  (relate_N m))

(cl:ensure-generic-function 'tar_lat-val :lambda-list '(m))
(cl:defmethod tar_lat-val ((m <pos_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:tar_lat-val is deprecated.  Use camera-msg:tar_lat instead.")
  (tar_lat m))

(cl:ensure-generic-function 'tar_lon-val :lambda-list '(m))
(cl:defmethod tar_lon-val ((m <pos_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:tar_lon-val is deprecated.  Use camera-msg:tar_lon instead.")
  (tar_lon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_message>) ostream)
  "Serializes a message object of type '<pos_message>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time) (cl:floor (cl:slot-value msg 'time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'relate_E))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'relate_N))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tar_lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tar_lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_message>) istream)
  "Deserializes a message object of type '<pos_message>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'relate_E) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'relate_N) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tar_lat) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tar_lon) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_message>)))
  "Returns string type for a message object of type '<pos_message>"
  "camera/pos_message")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_message)))
  "Returns string type for a message object of type 'pos_message"
  "camera/pos_message")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_message>)))
  "Returns md5sum for a message object of type '<pos_message>"
  "1d9e4de796d3278b9e92378ad269bd1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_message)))
  "Returns md5sum for a message object of type 'pos_message"
  "1d9e4de796d3278b9e92378ad269bd1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_message>)))
  "Returns full string definition for message of type '<pos_message>"
  (cl:format cl:nil "time time~%float32 relate_E #正东方向差距 单位 米~%float32 relate_N #正北方向差距 单位 米~%float32 tar_lat~%float32 tar_lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_message)))
  "Returns full string definition for message of type 'pos_message"
  (cl:format cl:nil "time time~%float32 relate_E #正东方向差距 单位 米~%float32 relate_N #正北方向差距 单位 米~%float32 tar_lat~%float32 tar_lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_message>))
  (cl:+ 0
     8
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_message>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_message
    (cl:cons ':time (time msg))
    (cl:cons ':relate_E (relate_E msg))
    (cl:cons ':relate_N (relate_N msg))
    (cl:cons ':tar_lat (tar_lat msg))
    (cl:cons ':tar_lon (tar_lon msg))
))
