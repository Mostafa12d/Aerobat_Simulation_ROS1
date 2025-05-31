;; Auto-generated. Do not edit!


(when (boundp 'ros_flappy_sim::CameraFrame)
  (if (not (find-package "ROS_FLAPPY_SIM"))
    (make-package "ROS_FLAPPY_SIM"))
  (shadow 'CameraFrame (find-package "ROS_FLAPPY_SIM")))
(unless (find-package "ROS_FLAPPY_SIM::CAMERAFRAME")
  (make-package "ROS_FLAPPY_SIM::CAMERAFRAME"))

(in-package "ROS")
;;//! \htmlinclude CameraFrame.msg.html


(defclass ros_flappy_sim::CameraFrame
  :super ros::object
  :slots (_header _width _height _step _data_length _data ))

(defmethod ros_flappy_sim::CameraFrame
  (:init
   (&key
    ((:header __header) "")
    ((:width __width) 0)
    ((:height __height) 0)
    ((:step __step) 0)
    ((:data_length __data_length) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header (string __header))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _step (round __step))
   (setq _data_length (round __data_length))
   (setq _data __data)
   self)
  (:header
   (&optional __header)
   (if __header (setq _header __header)) _header)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:data_length
   (&optional __data_length)
   (if __data_length (setq _data_length __data_length)) _data_length)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _header
    4 (length _header)
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; uint32 _step
    4
    ;; uint32 _data_length
    4
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _header
       (write-long (length _header) s) (princ _header s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _step
       (write-long _step s)
     ;; uint32 _data_length
       (write-long _data_length s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _header
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _header (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _step
     (setq _step (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _data_length
     (setq _data_length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ros_flappy_sim::CameraFrame :md5sum-) "c052b18bb3b86a43759aefbf998d6f5b")
(setf (get ros_flappy_sim::CameraFrame :datatype-) "ros_flappy_sim/CameraFrame")
(setf (get ros_flappy_sim::CameraFrame :definition-)
      "string header
uint32 width
uint32 height
uint32 step
uint32 data_length
uint8[] data
")



(provide :ros_flappy_sim/CameraFrame "c052b18bb3b86a43759aefbf998d6f5b")


