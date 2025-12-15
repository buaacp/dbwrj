; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude servo_msg.msg.html

(cl:defclass <servo_msg> (roslisp-msg-protocol:ros-message)
  ((angle1
    :reader angle1
    :initarg :angle1
    :type cl:float
    :initform 0.0)
   (angle2
    :reader angle2
    :initarg :angle2
    :type cl:float
    :initform 0.0))
)

(cl:defclass servo_msg (<servo_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <servo_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'servo_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<servo_msg> is deprecated: use camera-msg:servo_msg instead.")))

(cl:ensure-generic-function 'angle1-val :lambda-list '(m))
(cl:defmethod angle1-val ((m <servo_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:angle1-val is deprecated.  Use camera-msg:angle1 instead.")
  (angle1 m))

(cl:ensure-generic-function 'angle2-val :lambda-list '(m))
(cl:defmethod angle2-val ((m <servo_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:angle2-val is deprecated.  Use camera-msg:angle2 instead.")
  (angle2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <servo_msg>) ostream)
  "Serializes a message object of type '<servo_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <servo_msg>) istream)
  "Deserializes a message object of type '<servo_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<servo_msg>)))
  "Returns string type for a message object of type '<servo_msg>"
  "camera/servo_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'servo_msg)))
  "Returns string type for a message object of type 'servo_msg"
  "camera/servo_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<servo_msg>)))
  "Returns md5sum for a message object of type '<servo_msg>"
  "343bc431207c2d7a78fcf1a862aeef25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'servo_msg)))
  "Returns md5sum for a message object of type 'servo_msg"
  "343bc431207c2d7a78fcf1a862aeef25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<servo_msg>)))
  "Returns full string definition for message of type '<servo_msg>"
  (cl:format cl:nil "float32 angle1~%float32 angle2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'servo_msg)))
  "Returns full string definition for message of type 'servo_msg"
  (cl:format cl:nil "float32 angle1~%float32 angle2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <servo_msg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <servo_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'servo_msg
    (cl:cons ':angle1 (angle1 msg))
    (cl:cons ':angle2 (angle2 msg))
))
