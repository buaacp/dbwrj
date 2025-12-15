;; Auto-generated. Do not edit!


(when (boundp 'camera::servo_msg)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'servo_msg (find-package "CAMERA")))
(unless (find-package "CAMERA::SERVO_MSG")
  (make-package "CAMERA::SERVO_MSG"))

(in-package "ROS")
;;//! \htmlinclude servo_msg.msg.html


(defclass camera::servo_msg
  :super ros::object
  :slots (_angle1 _angle2 ))

(defmethod camera::servo_msg
  (:init
   (&key
    ((:angle1 __angle1) 0.0)
    ((:angle2 __angle2) 0.0)
    )
   (send-super :init)
   (setq _angle1 (float __angle1))
   (setq _angle2 (float __angle2))
   self)
  (:angle1
   (&optional __angle1)
   (if __angle1 (setq _angle1 __angle1)) _angle1)
  (:angle2
   (&optional __angle2)
   (if __angle2 (setq _angle2 __angle2)) _angle2)
  (:serialization-length
   ()
   (+
    ;; float32 _angle1
    4
    ;; float32 _angle2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle1
       (sys::poke _angle1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle2
       (sys::poke _angle2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle1
     (setq _angle1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle2
     (setq _angle2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get camera::servo_msg :md5sum-) "343bc431207c2d7a78fcf1a862aeef25")
(setf (get camera::servo_msg :datatype-) "camera/servo_msg")
(setf (get camera::servo_msg :definition-)
      "float32 angle1
float32 angle2
")



(provide :camera/servo_msg "343bc431207c2d7a78fcf1a862aeef25")


