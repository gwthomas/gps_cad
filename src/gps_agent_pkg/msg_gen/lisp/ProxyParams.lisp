; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude ProxyParams.msg.html

(cl:defclass <ProxyParams> (roslisp-msg-protocol:ros-message)
  ((dU
    :reader dU
    :initarg :dU
    :type cl:integer
    :initform 0))
)

(cl:defclass ProxyParams (<ProxyParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProxyParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProxyParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<ProxyParams> is deprecated: use gps_agent_pkg-msg:ProxyParams instead.")))

(cl:ensure-generic-function 'dU-val :lambda-list '(m))
(cl:defmethod dU-val ((m <ProxyParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:dU-val is deprecated.  Use gps_agent_pkg-msg:dU instead.")
  (dU m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProxyParams>) ostream)
  "Serializes a message object of type '<ProxyParams>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProxyParams>) istream)
  "Deserializes a message object of type '<ProxyParams>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProxyParams>)))
  "Returns string type for a message object of type '<ProxyParams>"
  "gps_agent_pkg/ProxyParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProxyParams)))
  "Returns string type for a message object of type 'ProxyParams"
  "gps_agent_pkg/ProxyParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProxyParams>)))
  "Returns md5sum for a message object of type '<ProxyParams>"
  "24131aa271a6e7d452bc4f62ea3b4c2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProxyParams)))
  "Returns md5sum for a message object of type 'ProxyParams"
  "24131aa271a6e7d452bc4f62ea3b4c2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProxyParams>)))
  "Returns full string definition for message of type '<ProxyParams>"
  (cl:format cl:nil "uint32 dU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProxyParams)))
  "Returns full string definition for message of type 'ProxyParams"
  (cl:format cl:nil "uint32 dU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProxyParams>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProxyParams>))
  "Converts a ROS message object to a list"
  (cl:list 'ProxyParams
    (cl:cons ':dU (dU msg))
))
