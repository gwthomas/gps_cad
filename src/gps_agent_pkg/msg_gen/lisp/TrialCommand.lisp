; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude TrialCommand.msg.html

(cl:defclass <TrialCommand> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (controller
    :reader controller
    :initarg :controller
    :type gps_agent_pkg-msg:ControllerParams
    :initform (cl:make-instance 'gps_agent_pkg-msg:ControllerParams))
   (T
    :reader T
    :initarg :T
    :type cl:integer
    :initform 0)
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0)
   (state_datatypes
    :reader state_datatypes
    :initarg :state_datatypes
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (obs_datatypes
    :reader obs_datatypes
    :initarg :obs_datatypes
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (ee_points
    :reader ee_points
    :initarg :ee_points
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ee_points_tgt
    :reader ee_points_tgt
    :initarg :ee_points_tgt
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TrialCommand (<TrialCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrialCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrialCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<TrialCommand> is deprecated: use gps_agent_pkg-msg:TrialCommand instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:id-val is deprecated.  Use gps_agent_pkg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:controller-val is deprecated.  Use gps_agent_pkg-msg:controller instead.")
  (controller m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:T-val is deprecated.  Use gps_agent_pkg-msg:T instead.")
  (T m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:frequency-val is deprecated.  Use gps_agent_pkg-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'state_datatypes-val :lambda-list '(m))
(cl:defmethod state_datatypes-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:state_datatypes-val is deprecated.  Use gps_agent_pkg-msg:state_datatypes instead.")
  (state_datatypes m))

(cl:ensure-generic-function 'obs_datatypes-val :lambda-list '(m))
(cl:defmethod obs_datatypes-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:obs_datatypes-val is deprecated.  Use gps_agent_pkg-msg:obs_datatypes instead.")
  (obs_datatypes m))

(cl:ensure-generic-function 'ee_points-val :lambda-list '(m))
(cl:defmethod ee_points-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:ee_points-val is deprecated.  Use gps_agent_pkg-msg:ee_points instead.")
  (ee_points m))

(cl:ensure-generic-function 'ee_points_tgt-val :lambda-list '(m))
(cl:defmethod ee_points_tgt-val ((m <TrialCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:ee_points_tgt-val is deprecated.  Use gps_agent_pkg-msg:ee_points_tgt instead.")
  (ee_points_tgt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrialCommand>) ostream)
  "Serializes a message object of type '<TrialCommand>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'controller) ostream)
  (cl:let* ((signed (cl:slot-value msg 'T)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state_datatypes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'state_datatypes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obs_datatypes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'obs_datatypes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ee_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'ee_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ee_points_tgt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'ee_points_tgt))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrialCommand>) istream)
  "Deserializes a message object of type '<TrialCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'controller) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'T) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state_datatypes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state_datatypes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obs_datatypes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obs_datatypes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ee_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ee_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ee_points_tgt) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ee_points_tgt)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrialCommand>)))
  "Returns string type for a message object of type '<TrialCommand>"
  "gps_agent_pkg/TrialCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrialCommand)))
  "Returns string type for a message object of type 'TrialCommand"
  "gps_agent_pkg/TrialCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrialCommand>)))
  "Returns md5sum for a message object of type '<TrialCommand>"
  "04466b7105be234f831db431c410ea7a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrialCommand)))
  "Returns md5sum for a message object of type 'TrialCommand"
  "04466b7105be234f831db431c410ea7a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrialCommand>)))
  "Returns full string definition for message of type '<TrialCommand>"
  (cl:format cl:nil "# This message is published to the C++ controller to start~%# a trial~%int32 id  # ID must be echoed back in SampleResult~%ControllerParams controller~%~%# Trial information~%int32 T  # Trajectory length~%float64 frequency  # Controller frequency~%int8[] state_datatypes  # Which data types to include in state~%int8[] obs_datatypes # Which data types to include in observation~%float64[] ee_points # A 3*n_points array containing offsets~%float64[] ee_points_tgt # A 3*n_points array containing the desired ee_points for this trial~%~%================================================================================~%MSG: gps_agent_pkg/ControllerParams~%int8 controller_to_execute  # controller enum, defined in gps_pb2~%~%CaffeParams caffe~%LinGaussParams lingauss~%TfParams tf~%~%================================================================================~%MSG: gps_agent_pkg/CaffeParams~%string net_param # Serialized net parameter with weights (equivalent of prototxt file)~%float32[] bias~%float32[] scale~%float32[] noise~%int32 dim_bias~%uint32 dU~%~%================================================================================~%MSG: gps_agent_pkg/LinGaussParams~%# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%================================================================================~%MSG: gps_agent_pkg/TfParams~%# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrialCommand)))
  "Returns full string definition for message of type 'TrialCommand"
  (cl:format cl:nil "# This message is published to the C++ controller to start~%# a trial~%int32 id  # ID must be echoed back in SampleResult~%ControllerParams controller~%~%# Trial information~%int32 T  # Trajectory length~%float64 frequency  # Controller frequency~%int8[] state_datatypes  # Which data types to include in state~%int8[] obs_datatypes # Which data types to include in observation~%float64[] ee_points # A 3*n_points array containing offsets~%float64[] ee_points_tgt # A 3*n_points array containing the desired ee_points for this trial~%~%================================================================================~%MSG: gps_agent_pkg/ControllerParams~%int8 controller_to_execute  # controller enum, defined in gps_pb2~%~%CaffeParams caffe~%LinGaussParams lingauss~%TfParams tf~%~%================================================================================~%MSG: gps_agent_pkg/CaffeParams~%string net_param # Serialized net parameter with weights (equivalent of prototxt file)~%float32[] bias~%float32[] scale~%float32[] noise~%int32 dim_bias~%uint32 dU~%~%================================================================================~%MSG: gps_agent_pkg/LinGaussParams~%# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%================================================================================~%MSG: gps_agent_pkg/TfParams~%# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrialCommand>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'controller))
     4
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state_datatypes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obs_datatypes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ee_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ee_points_tgt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrialCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TrialCommand
    (cl:cons ':id (id msg))
    (cl:cons ':controller (controller msg))
    (cl:cons ':T (T msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':state_datatypes (state_datatypes msg))
    (cl:cons ':obs_datatypes (obs_datatypes msg))
    (cl:cons ':ee_points (ee_points msg))
    (cl:cons ':ee_points_tgt (ee_points_tgt msg))
))
