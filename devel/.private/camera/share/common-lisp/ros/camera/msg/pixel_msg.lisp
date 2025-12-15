; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude pixel_msg.msg.html

(cl:defclass <pixel_msg> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (pixel_x
    :reader pixel_x
    :initarg :pixel_x
    :type cl:float
    :initform 0.0)
   (pixel_y
    :reader pixel_y
    :initarg :pixel_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass pixel_msg (<pixel_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pixel_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pixel_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<pixel_msg> is deprecated: use camera-msg:pixel_msg instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <pixel_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:time-val is deprecated.  Use camera-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'pixel_x-val :lambda-list '(m))
(cl:defmethod pixel_x-val ((m <pixel_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:pixel_x-val is deprecated.  Use camera-msg:pixel_x instead.")
  (pixel_x m))

(cl:ensure-generic-function 'pixel_y-val :lambda-list '(m))
(cl:defmethod pixel_y-val ((m <pixel_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:pixel_y-val is deprecated.  Use camera-msg:pixel_y instead.")
  (pixel_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pixel_msg>) ostream)
  "Serializes a message object of type '<pixel_msg>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pixel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pixel_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pixel_msg>) istream)
  "Deserializes a message object of type '<pixel_msg>"
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
    (cl:setf (cl:slot-value msg 'pixel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pixel_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pixel_msg>)))
  "Returns string type for a message object of type '<pixel_msg>"
  "camera/pixel_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pixel_msg)))
  "Returns string type for a message object of type 'pixel_msg"
  "camera/pixel_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pixel_msg>)))
  "Returns md5sum for a message object of type '<pixel_msg>"
  "92eab89a17307a1d310df233c30b69f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pixel_msg)))
  "Returns md5sum for a message object of type 'pixel_msg"
  "92eab89a17307a1d310df233c30b69f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pixel_msg>)))
  "Returns full string definition for message of type '<pixel_msg>"
  (cl:format cl:nil "time time~%float32 pixel_x~%float32 pixel_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pixel_msg)))
  "Returns full string definition for message of type 'pixel_msg"
  (cl:format cl:nil "time time~%float32 pixel_x~%float32 pixel_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pixel_msg>))
  (cl:+ 0
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pixel_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'pixel_msg
    (cl:cons ':time (time msg))
    (cl:cons ':pixel_x (pixel_x msg))
    (cl:cons ':pixel_y (pixel_y msg))
))
