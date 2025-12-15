;; Auto-generated. Do not edit!


(when (boundp 'camera::detect_msg)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'detect_msg (find-package "CAMERA")))
(unless (find-package "CAMERA::DETECT_MSG")
  (make-package "CAMERA::DETECT_MSG"))

(in-package "ROS")
;;//! \htmlinclude detect_msg.msg.html


(defclass camera::detect_msg
  :super ros::object
  :slots (_lat _lon ))

(defmethod camera::detect_msg
  (:init
   (&key
    ((:lat __lat) 0.0)
    ((:lon __lon) 0.0)
    )
   (send-super :init)
   (setq _lat (float __lat))
   (setq _lon (float __lon))
   self)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:lon
   (&optional __lon)
   (if __lon (setq _lon __lon)) _lon)
  (:serialization-length
   ()
   (+
    ;; float64 _lat
    8
    ;; float64 _lon
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lon
       (sys::poke _lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lon
     (setq _lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get camera::detect_msg :md5sum-) "deb12644498d4b5511a84dbd9af1e283")
(setf (get camera::detect_msg :datatype-) "camera/detect_msg")
(setf (get camera::detect_msg :definition-)
      "float64 lat
float64 lon

")



(provide :camera/detect_msg "deb12644498d4b5511a84dbd9af1e283")


