;; Auto-generated. Do not edit!


(when (boundp 'camera::pos_message)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'pos_message (find-package "CAMERA")))
(unless (find-package "CAMERA::POS_MESSAGE")
  (make-package "CAMERA::POS_MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude pos_message.msg.html


(defclass camera::pos_message
  :super ros::object
  :slots (_time _relate_E _relate_N _tar_lat _tar_lon ))

(defmethod camera::pos_message
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:relate_E __relate_E) 0.0)
    ((:relate_N __relate_N) 0.0)
    ((:tar_lat __tar_lat) 0.0)
    ((:tar_lon __tar_lon) 0.0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _relate_E (float __relate_E))
   (setq _relate_N (float __relate_N))
   (setq _tar_lat (float __tar_lat))
   (setq _tar_lon (float __tar_lon))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:relate_E
   (&optional __relate_E)
   (if __relate_E (setq _relate_E __relate_E)) _relate_E)
  (:relate_N
   (&optional __relate_N)
   (if __relate_N (setq _relate_N __relate_N)) _relate_N)
  (:tar_lat
   (&optional __tar_lat)
   (if __tar_lat (setq _tar_lat __tar_lat)) _tar_lat)
  (:tar_lon
   (&optional __tar_lon)
   (if __tar_lon (setq _tar_lon __tar_lon)) _tar_lon)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; float32 _relate_E
    4
    ;; float32 _relate_N
    4
    ;; float32 _tar_lat
    4
    ;; float32 _tar_lon
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; float32 _relate_E
       (sys::poke _relate_E (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _relate_N
       (sys::poke _relate_N (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tar_lat
       (sys::poke _tar_lat (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tar_lon
       (sys::poke _tar_lon (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _relate_E
     (setq _relate_E (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _relate_N
     (setq _relate_N (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tar_lat
     (setq _tar_lat (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tar_lon
     (setq _tar_lon (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get camera::pos_message :md5sum-) "1d9e4de796d3278b9e92378ad269bd1e")
(setf (get camera::pos_message :datatype-) "camera/pos_message")
(setf (get camera::pos_message :definition-)
      "time time
float32 relate_E #正东方向差距 单位 米
float32 relate_N #正北方向差距 单位 米
float32 tar_lat
float32 tar_lon
")



(provide :camera/pos_message "1d9e4de796d3278b9e92378ad269bd1e")


