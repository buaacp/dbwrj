
(cl:in-package :asdf)

(defsystem "camera-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "detect_msg" :depends-on ("_package_detect_msg"))
    (:file "_package_detect_msg" :depends-on ("_package"))
    (:file "gimbal_msg" :depends-on ("_package_gimbal_msg"))
    (:file "_package_gimbal_msg" :depends-on ("_package"))
    (:file "neighbor_camera_data" :depends-on ("_package_neighbor_camera_data"))
    (:file "_package_neighbor_camera_data" :depends-on ("_package"))
    (:file "pixel_msg" :depends-on ("_package_pixel_msg"))
    (:file "_package_pixel_msg" :depends-on ("_package"))
    (:file "pos_message" :depends-on ("_package_pos_message"))
    (:file "_package_pos_message" :depends-on ("_package"))
    (:file "servo_msg" :depends-on ("_package_servo_msg"))
    (:file "_package_servo_msg" :depends-on ("_package"))
  ))