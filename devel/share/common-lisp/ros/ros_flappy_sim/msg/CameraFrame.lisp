; Auto-generated. Do not edit!


(cl:in-package ros_flappy_sim-msg)


;//! \htmlinclude CameraFrame.msg.html

(cl:defclass <CameraFrame> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type cl:string
    :initform "")
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (step
    :reader step
    :initarg :step
    :type cl:integer
    :initform 0)
   (data_length
    :reader data_length
    :initarg :data_length
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass CameraFrame (<CameraFrame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraFrame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraFrame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_flappy_sim-msg:<CameraFrame> is deprecated: use ros_flappy_sim-msg:CameraFrame instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:header-val is deprecated.  Use ros_flappy_sim-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:width-val is deprecated.  Use ros_flappy_sim-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:height-val is deprecated.  Use ros_flappy_sim-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:step-val is deprecated.  Use ros_flappy_sim-msg:step instead.")
  (step m))

(cl:ensure-generic-function 'data_length-val :lambda-list '(m))
(cl:defmethod data_length-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:data_length-val is deprecated.  Use ros_flappy_sim-msg:data_length instead.")
  (data_length m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <CameraFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_flappy_sim-msg:data-val is deprecated.  Use ros_flappy_sim-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraFrame>) ostream)
  "Serializes a message object of type '<CameraFrame>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'header))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'header))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'data_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'data_length)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraFrame>) istream)
  "Deserializes a message object of type '<CameraFrame>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'header) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'header) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraFrame>)))
  "Returns string type for a message object of type '<CameraFrame>"
  "ros_flappy_sim/CameraFrame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraFrame)))
  "Returns string type for a message object of type 'CameraFrame"
  "ros_flappy_sim/CameraFrame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraFrame>)))
  "Returns md5sum for a message object of type '<CameraFrame>"
  "c052b18bb3b86a43759aefbf998d6f5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraFrame)))
  "Returns md5sum for a message object of type 'CameraFrame"
  "c052b18bb3b86a43759aefbf998d6f5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraFrame>)))
  "Returns full string definition for message of type '<CameraFrame>"
  (cl:format cl:nil "string header~%uint32 width~%uint32 height~%uint32 step~%uint32 data_length~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraFrame)))
  "Returns full string definition for message of type 'CameraFrame"
  (cl:format cl:nil "string header~%uint32 width~%uint32 height~%uint32 step~%uint32 data_length~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraFrame>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraFrame>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraFrame
    (cl:cons ':header (header msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':step (step msg))
    (cl:cons ':data_length (data_length msg))
    (cl:cons ':data (data msg))
))
