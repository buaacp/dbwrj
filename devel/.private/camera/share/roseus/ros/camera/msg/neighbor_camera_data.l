;; Auto-generated. Do not edit!


(when (boundp 'camera::neighbor_camera_data)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'neighbor_camera_data (find-package "CAMERA")))
(unless (find-package "CAMERA::NEIGHBOR_CAMERA_DATA")
  (make-package "CAMERA::NEIGHBOR_CAMERA_DATA"))

(in-package "ROS")
;;//! \htmlinclude neighbor_camera_data.msg.html


(defclass camera::neighbor_camera_data
  :super ros::object
  :slots (_UAV_ID _UAV_OBJ_X _UAV_OBJ_Y _latitude _longitude _altitude ))

(defmethod camera::neighbor_camera_data
  (:init
   (&key
    ((:UAV_ID __UAV_ID) 0)
    ((:UAV_OBJ_X __UAV_OBJ_X) 0)
    ((:UAV_OBJ_Y __UAV_OBJ_Y) 0)
    ((:latitude __latitude) 0)
    ((:longitude __longitude) 0)
    ((:altitude __altitude) 0)
    )
   (send-super :init)
   (setq _UAV_ID (round __UAV_ID))
   (setq _UAV_OBJ_X (round __UAV_OBJ_X))
   (setq _UAV_OBJ_Y (round __UAV_OBJ_Y))
   (setq _latitude (round __latitude))
   (setq _longitude (round __longitude))
   (setq _altitude (round __altitude))
   self)
  (:UAV_ID
   (&optional __UAV_ID)
   (if __UAV_ID (setq _UAV_ID __UAV_ID)) _UAV_ID)
  (:UAV_OBJ_X
   (&optional __UAV_OBJ_X)
   (if __UAV_OBJ_X (setq _UAV_OBJ_X __UAV_OBJ_X)) _UAV_OBJ_X)
  (:UAV_OBJ_Y
   (&optional __UAV_OBJ_Y)
   (if __UAV_OBJ_Y (setq _UAV_OBJ_Y __UAV_OBJ_Y)) _UAV_OBJ_Y)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:serialization-length
   ()
   (+
    ;; uint8 _UAV_ID
    1
    ;; uint16 _UAV_OBJ_X
    2
    ;; uint16 _UAV_OBJ_Y
    2
    ;; uint32 _latitude
    4
    ;; uint32 _longitude
    4
    ;; uint16 _altitude
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _UAV_ID
       (write-byte _UAV_ID s)
     ;; uint16 _UAV_OBJ_X
       (write-word _UAV_OBJ_X s)
     ;; uint16 _UAV_OBJ_Y
       (write-word _UAV_OBJ_Y s)
     ;; uint32 _latitude
       (write-long _latitude s)
     ;; uint32 _longitude
       (write-long _longitude s)
     ;; uint16 _altitude
       (write-word _altitude s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _UAV_ID
     (setq _UAV_ID (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _UAV_OBJ_X
     (setq _UAV_OBJ_X (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _UAV_OBJ_Y
     (setq _UAV_OBJ_Y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _latitude
     (setq _latitude (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _longitude
     (setq _longitude (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _altitude
     (setq _altitude (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get camera::neighbor_camera_data :md5sum-) "cf9983e93e6a13e8480601768a9c54d4")
(setf (get camera::neighbor_camera_data :datatype-) "camera/neighbor_camera_data")
(setf (get camera::neighbor_camera_data :definition-)
      "uint8   UAV_ID
#物体相对位置估计
uint16 UAV_OBJ_X
uint16 UAV_OBJ_Y
#无人机信息
uint32 latitude
uint32 longitude
uint16 altitude
")



(provide :camera/neighbor_camera_data "cf9983e93e6a13e8480601768a9c54d4")


