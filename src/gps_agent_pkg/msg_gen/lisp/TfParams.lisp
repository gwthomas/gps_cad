; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude TfParams.msg.html

(cl:defclass <TfParams> (roslisp-msg-protocol:ros-message)
  ((dU
    :reader dU
    :initarg :dU
    :type cl:integer
    :initform 0))
)

(cl:defclass TfParams (<TfParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TfParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TfParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<TfParams> is deprecated: use gps_agent_pkg-msg:TfParams instead.")))

(cl:ensure-generic-function 'dU-val :lambda-list '(m))
(cl:defmethod dU-val ((m <TfParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:dU-val is deprecated.  Use gps_agent_pkg-msg:dU instead.")
  (dU m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TfParams>) ostream)
  "Serializes a message object of type '<TfParams>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TfParams>) istream)
  "Deserializes a message object of type '<TfParams>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TfParams>)))
  "Returns string type for a message object of type '<TfParams>"
  "gps_agent_pkg/TfParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TfParams)))
  "Returns string type for a message object of type 'TfParams"
  "gps_agent_pkg/TfParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TfParams>)))
  "Returns md5sum for a message object of type '<TfParams>"
  "24131aa271a6e7d452bc4f62ea3b4c2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TfParams)))
  "Returns md5sum for a message object of type 'TfParams"
  "24131aa271a6e7d452bc4f62ea3b4c2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TfParams>)))
  "Returns full string definition for message of type '<TfParams>"
  (cl:format cl:nil "# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TfParams)))
  "Returns full string definition for message of type 'TfParams"
  (cl:format cl:nil "# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TfParams>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TfParams>))
  "Converts a ROS message object to a list"
  (cl:list 'TfParams
    (cl:cons ':dU (dU msg))
))
