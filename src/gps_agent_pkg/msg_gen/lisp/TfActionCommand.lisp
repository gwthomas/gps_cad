; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude TfActionCommand.msg.html

(cl:defclass <TfActionCommand> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (dU
    :reader dU
    :initarg :dU
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass TfActionCommand (<TfActionCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TfActionCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TfActionCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<TfActionCommand> is deprecated: use gps_agent_pkg-msg:TfActionCommand instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <TfActionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:action-val is deprecated.  Use gps_agent_pkg-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'dU-val :lambda-list '(m))
(cl:defmethod dU-val ((m <TfActionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:dU-val is deprecated.  Use gps_agent_pkg-msg:dU instead.")
  (dU m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TfActionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:id-val is deprecated.  Use gps_agent_pkg-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TfActionCommand>) ostream)
  "Serializes a message object of type '<TfActionCommand>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'action))))
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
   (cl:slot-value msg 'action))
  (cl:let* ((signed (cl:slot-value msg 'dU)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TfActionCommand>) istream)
  "Deserializes a message object of type '<TfActionCommand>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'action) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'action)))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dU) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TfActionCommand>)))
  "Returns string type for a message object of type '<TfActionCommand>"
  "gps_agent_pkg/TfActionCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TfActionCommand)))
  "Returns string type for a message object of type 'TfActionCommand"
  "gps_agent_pkg/TfActionCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TfActionCommand>)))
  "Returns md5sum for a message object of type '<TfActionCommand>"
  "f309f95a21bb20b690ca72e110126ceb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TfActionCommand)))
  "Returns md5sum for a message object of type 'TfActionCommand"
  "f309f95a21bb20b690ca72e110126ceb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TfActionCommand>)))
  "Returns full string definition for message of type '<TfActionCommand>"
  (cl:format cl:nil "float64[] action~%int32 dU~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TfActionCommand)))
  "Returns full string definition for message of type 'TfActionCommand"
  (cl:format cl:nil "float64[] action~%int32 dU~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TfActionCommand>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'action) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TfActionCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TfActionCommand
    (cl:cons ':action (action msg))
    (cl:cons ':dU (dU msg))
    (cl:cons ':id (id msg))
))
