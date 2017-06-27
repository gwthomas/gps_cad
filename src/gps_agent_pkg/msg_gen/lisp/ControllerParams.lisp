; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude ControllerParams.msg.html

(cl:defclass <ControllerParams> (roslisp-msg-protocol:ros-message)
  ((controller_to_execute
    :reader controller_to_execute
    :initarg :controller_to_execute
    :type cl:fixnum
    :initform 0)
   (caffe
    :reader caffe
    :initarg :caffe
    :type gps_agent_pkg-msg:CaffeParams
    :initform (cl:make-instance 'gps_agent_pkg-msg:CaffeParams))
   (lingauss
    :reader lingauss
    :initarg :lingauss
    :type gps_agent_pkg-msg:LinGaussParams
    :initform (cl:make-instance 'gps_agent_pkg-msg:LinGaussParams))
   (tf
    :reader tf
    :initarg :tf
    :type gps_agent_pkg-msg:TfParams
    :initform (cl:make-instance 'gps_agent_pkg-msg:TfParams)))
)

(cl:defclass ControllerParams (<ControllerParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<ControllerParams> is deprecated: use gps_agent_pkg-msg:ControllerParams instead.")))

(cl:ensure-generic-function 'controller_to_execute-val :lambda-list '(m))
(cl:defmethod controller_to_execute-val ((m <ControllerParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:controller_to_execute-val is deprecated.  Use gps_agent_pkg-msg:controller_to_execute instead.")
  (controller_to_execute m))

(cl:ensure-generic-function 'caffe-val :lambda-list '(m))
(cl:defmethod caffe-val ((m <ControllerParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:caffe-val is deprecated.  Use gps_agent_pkg-msg:caffe instead.")
  (caffe m))

(cl:ensure-generic-function 'lingauss-val :lambda-list '(m))
(cl:defmethod lingauss-val ((m <ControllerParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:lingauss-val is deprecated.  Use gps_agent_pkg-msg:lingauss instead.")
  (lingauss m))

(cl:ensure-generic-function 'tf-val :lambda-list '(m))
(cl:defmethod tf-val ((m <ControllerParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:tf-val is deprecated.  Use gps_agent_pkg-msg:tf instead.")
  (tf m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerParams>) ostream)
  "Serializes a message object of type '<ControllerParams>"
  (cl:let* ((signed (cl:slot-value msg 'controller_to_execute)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'caffe) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lingauss) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tf) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerParams>) istream)
  "Deserializes a message object of type '<ControllerParams>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller_to_execute) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'caffe) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lingauss) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tf) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerParams>)))
  "Returns string type for a message object of type '<ControllerParams>"
  "gps_agent_pkg/ControllerParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerParams)))
  "Returns string type for a message object of type 'ControllerParams"
  "gps_agent_pkg/ControllerParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerParams>)))
  "Returns md5sum for a message object of type '<ControllerParams>"
  "b21eed6449b84a548fab33a89f3b3c3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerParams)))
  "Returns md5sum for a message object of type 'ControllerParams"
  "b21eed6449b84a548fab33a89f3b3c3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerParams>)))
  "Returns full string definition for message of type '<ControllerParams>"
  (cl:format cl:nil "int8 controller_to_execute  # controller enum, defined in gps_pb2~%~%CaffeParams caffe~%LinGaussParams lingauss~%TfParams tf~%~%================================================================================~%MSG: gps_agent_pkg/CaffeParams~%string net_param # Serialized net parameter with weights (equivalent of prototxt file)~%float32[] bias~%float32[] scale~%float32[] noise~%int32 dim_bias~%uint32 dU~%~%================================================================================~%MSG: gps_agent_pkg/LinGaussParams~%# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%================================================================================~%MSG: gps_agent_pkg/TfParams~%# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerParams)))
  "Returns full string definition for message of type 'ControllerParams"
  (cl:format cl:nil "int8 controller_to_execute  # controller enum, defined in gps_pb2~%~%CaffeParams caffe~%LinGaussParams lingauss~%TfParams tf~%~%================================================================================~%MSG: gps_agent_pkg/CaffeParams~%string net_param # Serialized net parameter with weights (equivalent of prototxt file)~%float32[] bias~%float32[] scale~%float32[] noise~%int32 dim_bias~%uint32 dU~%~%================================================================================~%MSG: gps_agent_pkg/LinGaussParams~%# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%================================================================================~%MSG: gps_agent_pkg/TfParams~%# Tf Params. just need to track dU.~%uint32 dU~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerParams>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'caffe))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lingauss))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tf))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerParams>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerParams
    (cl:cons ':controller_to_execute (controller_to_execute msg))
    (cl:cons ':caffe (caffe msg))
    (cl:cons ':lingauss (lingauss msg))
    (cl:cons ':tf (tf msg))
))
