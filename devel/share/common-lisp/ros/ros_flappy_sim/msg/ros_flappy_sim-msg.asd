
(cl:in-package :asdf)

(defsystem "ros_flappy_sim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Axes" :depends-on ("_package_Axes"))
    (:file "_package_Axes" :depends-on ("_package"))
    (:file "CameraFrame" :depends-on ("_package_CameraFrame"))
    (:file "_package_CameraFrame" :depends-on ("_package"))
  ))