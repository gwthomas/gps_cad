; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude TfObsData.msg.html

(cl:defclass <TfObsData> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (shape
    :reader shape
    :initarg :shape
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass TfObsData (<TfObsData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TfObsData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TfObsData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<TfObsData> is deprecated: use gps_agent_pkg-msg:TfObsData instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <TfObsData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:data-val is deprecated.  Use gps_agent_pkg-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <TfObsData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:shape-val is deprecated.  Use gps_agent_pkg-msg:shape instead.")
  (shape m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TfObsData>) ostream)
  "Serializes a message object of type '<TfObsData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
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
   (cl:slot-value msg 'data))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'shape))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'shape))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TfObsData>) istream)
  "Deserializes a message object of type '<TfObsData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
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
  (cl:setf (cl:slot-value msg 'shape) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'shape)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TfObsData>)))
  "Returns string type for a message object of type '<TfObsData>"
  "gps_agent_pkg/TfObsData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TfObsData)))
  "Returns string type for a message object of type 'TfObsData"
  "gps_agent_pkg/TfObsData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TfObsData>)))
  "Returns md5sum for a message object of type '<TfObsData>"
  "1c5d3c77f5a9734934108f61a36f5512")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TfObsData)))
  "Returns md5sum for a message object of type 'TfObsData"
  "1c5d3c77f5a9734934108f61a36f5512")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TfObsData>)))
  "Returns full string definition for message of type '<TfObsData>"
  (cl:format cl:nil "float64[] data~%int32[] shape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TfObsData)))
  "Returns full string definition for message of type 'TfObsData"
  (cl:format cl:nil "float64[] data~%int32[] shape~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TfObsData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'shape) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TfObsData>))
  "Converts a ROS message object to a list"
  (cl:list 'TfObsData
    (cl:cons ':data (data msg))
    (cl:cons ':shape (shape msg))
))
