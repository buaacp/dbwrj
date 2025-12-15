; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude gimbal_msg.msg.html

(cl:defclass <gimbal_msg> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass gimbal_msg (<gimbal_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gimbal_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gimbal_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<gimbal_msg> is deprecated: use camera-msg:gimbal_msg instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <gimbal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:pos_x-val is deprecated.  Use camera-msg:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <gimbal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:pos_y-val is deprecated.  Use camera-msg:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <gimbal_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:height-val is deprecated.  Use camera-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gimbal_msg>) ostream)
  "Serializes a message object of type '<gimbal_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gimbal_msg>) istream)
  "Deserializes a message object of type '<gimbal_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gimbal_msg>)))
  "Returns string type for a message object of type '<gimbal_msg>"
  "camera/gimbal_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gimbal_msg)))
  "Returns string type for a message object of type 'gimbal_msg"
  "camera/gimbal_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gimbal_msg>)))
  "Returns md5sum for a message object of type '<gimbal_msg>"
  "ef9bdc0e9b0547b3ddcf9b4fb816ef5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gimbal_msg)))
  "Returns md5sum for a message object of type 'gimbal_msg"
  "ef9bdc0e9b0547b3ddcf9b4fb816ef5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gimbal_msg>)))
  "Returns full string definition for message of type '<gimbal_msg>"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%float32 height~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gimbal_msg)))
  "Returns full string definition for message of type 'gimbal_msg"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%float32 height~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gimbal_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gimbal_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'gimbal_msg
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':height (height msg))
))
