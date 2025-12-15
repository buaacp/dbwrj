;; Auto-generated. Do not edit!


(when (boundp 'camera::gimbal_msg)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'gimbal_msg (find-package "CAMERA")))
(unless (find-package "CAMERA::GIMBAL_MSG")
  (make-package "CAMERA::GIMBAL_MSG"))

(in-package "ROS")
;;//! \htmlinclude gimbal_msg.msg.html


(defclass camera::gimbal_msg
  :super ros::object
  :slots (_pos_x _pos_y _height ))

(defmethod camera::gimbal_msg
  (:init
   (&key
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   (setq _height (float __height))
   self)
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float32 _pos_x
    4
    ;; float32 _pos_y
    4
    ;; float32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pos_x
     (setq _pos_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pos_y
     (setq _pos_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get camera::gimbal_msg :md5sum-) "ef9bdc0e9b0547b3ddcf9b4fb816ef5f")
(setf (get camera::gimbal_msg :datatype-) "camera/gimbal_msg")
(setf (get camera::gimbal_msg :definition-)
      "float32 pos_x
float32 pos_y
float32 height
")



(provide :camera/gimbal_msg "ef9bdc0e9b0547b3ddcf9b4fb816ef5f")


