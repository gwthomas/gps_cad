; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-srv)


;//! \htmlinclude ProxyControl-request.msg.html

(cl:defclass <ProxyControl-request> (roslisp-msg-protocol:ros-message)
  ((obs
    :reader obs
    :initarg :obs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ProxyControl-request (<ProxyControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProxyControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProxyControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-srv:<ProxyControl-request> is deprecated: use gps_agent_pkg-srv:ProxyControl-request instead.")))

(cl:ensure-generic-function 'obs-val :lambda-list '(m))
(cl:defmethod obs-val ((m <ProxyControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-srv:obs-val is deprecated.  Use gps_agent_pkg-srv:obs instead.")
  (obs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProxyControl-request>) ostream)
  "Serializes a message object of type '<ProxyControl-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obs))))
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
   (cl:slot-value msg 'obs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProxyControl-request>) istream)
  "Deserializes a message object of type '<ProxyControl-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obs)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProxyControl-request>)))
  "Returns string type for a service object of type '<ProxyControl-request>"
  "gps_agent_pkg/ProxyControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProxyControl-request)))
  "Returns string type for a service object of type 'ProxyControl-request"
  "gps_agent_pkg/ProxyControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProxyControl-request>)))
  "Returns md5sum for a message object of type '<ProxyControl-request>"
  "df8911a3dc99abf752eaebab6600df26")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProxyControl-request)))
  "Returns md5sum for a message object of type 'ProxyControl-request"
  "df8911a3dc99abf752eaebab6600df26")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProxyControl-request>)))
  "Returns full string definition for message of type '<ProxyControl-request>"
  (cl:format cl:nil "float64[] obs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProxyControl-request)))
  "Returns full string definition for message of type 'ProxyControl-request"
  (cl:format cl:nil "float64[] obs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProxyControl-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProxyControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ProxyControl-request
    (cl:cons ':obs (obs msg))
))
;//! \htmlinclude ProxyControl-response.msg.html

(cl:defclass <ProxyControl-response> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ProxyControl-response (<ProxyControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProxyControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProxyControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-srv:<ProxyControl-response> is deprecated: use gps_agent_pkg-srv:ProxyControl-response instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <ProxyControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-srv:action-val is deprecated.  Use gps_agent_pkg-srv:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProxyControl-response>) ostream)
  "Serializes a message object of type '<ProxyControl-response>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProxyControl-response>) istream)
  "Deserializes a message object of type '<ProxyControl-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProxyControl-response>)))
  "Returns string type for a service object of type '<ProxyControl-response>"
  "gps_agent_pkg/ProxyControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProxyControl-response)))
  "Returns string type for a service object of type 'ProxyControl-response"
  "gps_agent_pkg/ProxyControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProxyControl-response>)))
  "Returns md5sum for a message object of type '<ProxyControl-response>"
  "df8911a3dc99abf752eaebab6600df26")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProxyControl-response)))
  "Returns md5sum for a message object of type 'ProxyControl-response"
  "df8911a3dc99abf752eaebab6600df26")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProxyControl-response>)))
  "Returns full string definition for message of type '<ProxyControl-response>"
  (cl:format cl:nil "float64[] action~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProxyControl-response)))
  "Returns full string definition for message of type 'ProxyControl-response"
  (cl:format cl:nil "float64[] action~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProxyControl-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'action) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProxyControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ProxyControl-response
    (cl:cons ':action (action msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ProxyControl)))
  'ProxyControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ProxyControl)))
  'ProxyControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProxyControl)))
  "Returns string type for a service object of type '<ProxyControl>"
  "gps_agent_pkg/ProxyControl")