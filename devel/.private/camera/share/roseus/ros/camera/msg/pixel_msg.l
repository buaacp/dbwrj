;; Auto-generated. Do not edit!


(when (boundp 'camera::pixel_msg)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'pixel_msg (find-package "CAMERA")))
(unless (find-package "CAMERA::PIXEL_MSG")
  (make-package "CAMERA::PIXEL_MSG"))

(in-package "ROS")
;;//! \htmlinclude pixel_msg.msg.html


(defclass camera::pixel_msg
  :super ros::object
  :slots (_time _pixel_x _pixel_y ))

(defmethod camera::pixel_msg
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:pixel_x __pixel_x) 0.0)
    ((:pixel_y __pixel_y) 0.0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _pixel_x (float __pixel_x))
   (setq _pixel_y (float __pixel_y))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:pixel_x
   (&optional __pixel_x)
   (if __pixel_x (setq _pixel_x __pixel_x)) _pixel_x)
  (:pixel_y
   (&optional __pixel_y)
   (if __pixel_y (setq _pixel_y __pixel_y)) _pixel_y)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; float32 _pixel_x
    4
    ;; float32 _pixel_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; float32 _pixel_x
       (sys::poke _pixel_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pixel_y
       (sys::poke _pixel_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _pixel_x
     (setq _pixel_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pixel_y
     (setq _pixel_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get camera::pixel_msg :md5sum-) "92eab89a17307a1d310df233c30b69f6")
(setf (get camera::pixel_msg :datatype-) "camera/pixel_msg")
(setf (get camera::pixel_msg :definition-)
      "time time
float32 pixel_x
float32 pixel_y
")



(provide :camera/pixel_msg "92eab89a17307a1d310df233c30b69f6")


