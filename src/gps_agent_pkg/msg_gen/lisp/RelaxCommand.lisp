; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude RelaxCommand.msg.html

(cl:defclass <RelaxCommand> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (arm
    :reader arm
    :initarg :arm
    :type cl:integer
    :initform 0))
)

(cl:defclass RelaxCommand (<RelaxCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelaxCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelaxCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<RelaxCommand> is deprecated: use gps_agent_pkg-msg:RelaxCommand instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <RelaxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:stamp-val is deprecated.  Use gps_agent_pkg-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RelaxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:id-val is deprecated.  Use gps_agent_pkg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <RelaxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:arm-val is deprecated.  Use gps_agent_pkg-msg:arm instead.")
  (arm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelaxCommand>) ostream)
  "Serializes a message object of type '<RelaxCommand>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelaxCommand>) istream)
  "Deserializes a message object of type '<RelaxCommand>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelaxCommand>)))
  "Returns string type for a message object of type '<RelaxCommand>"
  "gps_agent_pkg/RelaxCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelaxCommand)))
  "Returns string type for a message object of type 'RelaxCommand"
  "gps_agent_pkg/RelaxCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelaxCommand>)))
  "Returns md5sum for a message object of type '<RelaxCommand>"
  "08fa438768d567bd060ce1ef6e4edc87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelaxCommand)))
  "Returns md5sum for a message object of type 'RelaxCommand"
  "08fa438768d567bd060ce1ef6e4edc87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelaxCommand>)))
  "Returns full string definition for message of type '<RelaxCommand>"
  (cl:format cl:nil "time stamp~%int32 id  # ID must be echoed back in SampleResult~%int32 arm  # which arm to relax (ActuatorType enum)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelaxCommand)))
  "Returns full string definition for message of type 'RelaxCommand"
  (cl:format cl:nil "time stamp~%int32 id  # ID must be echoed back in SampleResult~%int32 arm  # which arm to relax (ActuatorType enum)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelaxCommand>))
  (cl:+ 0
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelaxCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'RelaxCommand
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':id (id msg))
    (cl:cons ':arm (arm msg))
))
