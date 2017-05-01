; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude SampleResult.msg.html

(cl:defclass <SampleResult> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (sensor_data
    :reader sensor_data
    :initarg :sensor_data
    :type (cl:vector gps_agent_pkg-msg:DataType)
   :initform (cl:make-array 0 :element-type 'gps_agent_pkg-msg:DataType :initial-element (cl:make-instance 'gps_agent_pkg-msg:DataType))))
)

(cl:defclass SampleResult (<SampleResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SampleResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SampleResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<SampleResult> is deprecated: use gps_agent_pkg-msg:SampleResult instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SampleResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:id-val is deprecated.  Use gps_agent_pkg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'sensor_data-val :lambda-list '(m))
(cl:defmethod sensor_data-val ((m <SampleResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:sensor_data-val is deprecated.  Use gps_agent_pkg-msg:sensor_data instead.")
  (sensor_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SampleResult>) ostream)
  "Serializes a message object of type '<SampleResult>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sensor_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SampleResult>) istream)
  "Deserializes a message object of type '<SampleResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'gps_agent_pkg-msg:DataType))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SampleResult>)))
  "Returns string type for a message object of type '<SampleResult>"
  "gps_agent_pkg/SampleResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SampleResult)))
  "Returns string type for a message object of type 'SampleResult"
  "gps_agent_pkg/SampleResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SampleResult>)))
  "Returns md5sum for a message object of type '<SampleResult>"
  "c54dfe2af7c83bf41623e1a91dae494d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SampleResult)))
  "Returns md5sum for a message object of type 'SampleResult"
  "c54dfe2af7c83bf41623e1a91dae494d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SampleResult>)))
  "Returns full string definition for message of type '<SampleResult>"
  (cl:format cl:nil "int32 id~%~%# Contains everything needed to reconstruct a Sample object~%DataType[] sensor_data~%~%================================================================================~%MSG: gps_agent_pkg/DataType~%int8 data_type  # enum of sample type of requested data, defined in gps_pb2~%float64[] data~%int32[] shape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SampleResult)))
  "Returns full string definition for message of type 'SampleResult"
  (cl:format cl:nil "int32 id~%~%# Contains everything needed to reconstruct a Sample object~%DataType[] sensor_data~%~%================================================================================~%MSG: gps_agent_pkg/DataType~%int8 data_type  # enum of sample type of requested data, defined in gps_pb2~%float64[] data~%int32[] shape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SampleResult>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SampleResult>))
  "Converts a ROS message object to a list"
  (cl:list 'SampleResult
    (cl:cons ':id (id msg))
    (cl:cons ':sensor_data (sensor_data msg))
))
