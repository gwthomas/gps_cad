; Auto-generated. Do not edit!


(cl:in-package gps_agent_pkg-msg)


;//! \htmlinclude LinGaussParams.msg.html

(cl:defclass <LinGaussParams> (roslisp-msg-protocol:ros-message)
  ((dX
    :reader dX
    :initarg :dX
    :type cl:integer
    :initform 0)
   (dU
    :reader dU
    :initarg :dU
    :type cl:integer
    :initform 0)
   (K_t
    :reader K_t
    :initarg :K_t
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (k_t
    :reader k_t
    :initarg :k_t
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass LinGaussParams (<LinGaussParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinGaussParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinGaussParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gps_agent_pkg-msg:<LinGaussParams> is deprecated: use gps_agent_pkg-msg:LinGaussParams instead.")))

(cl:ensure-generic-function 'dX-val :lambda-list '(m))
(cl:defmethod dX-val ((m <LinGaussParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:dX-val is deprecated.  Use gps_agent_pkg-msg:dX instead.")
  (dX m))

(cl:ensure-generic-function 'dU-val :lambda-list '(m))
(cl:defmethod dU-val ((m <LinGaussParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:dU-val is deprecated.  Use gps_agent_pkg-msg:dU instead.")
  (dU m))

(cl:ensure-generic-function 'K_t-val :lambda-list '(m))
(cl:defmethod K_t-val ((m <LinGaussParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:K_t-val is deprecated.  Use gps_agent_pkg-msg:K_t instead.")
  (K_t m))

(cl:ensure-generic-function 'k_t-val :lambda-list '(m))
(cl:defmethod k_t-val ((m <LinGaussParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gps_agent_pkg-msg:k_t-val is deprecated.  Use gps_agent_pkg-msg:k_t instead.")
  (k_t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinGaussParams>) ostream)
  "Serializes a message object of type '<LinGaussParams>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'K_t))))
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
   (cl:slot-value msg 'K_t))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'k_t))))
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
   (cl:slot-value msg 'k_t))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinGaussParams>) istream)
  "Deserializes a message object of type '<LinGaussParams>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dU)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dU)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'K_t) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'K_t)))
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
  (cl:setf (cl:slot-value msg 'k_t) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'k_t)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinGaussParams>)))
  "Returns string type for a message object of type '<LinGaussParams>"
  "gps_agent_pkg/LinGaussParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinGaussParams)))
  "Returns string type for a message object of type 'LinGaussParams"
  "gps_agent_pkg/LinGaussParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinGaussParams>)))
  "Returns md5sum for a message object of type '<LinGaussParams>"
  "d59b25753e6e310cbad655a7176ffe29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinGaussParams)))
  "Returns md5sum for a message object of type 'LinGaussParams"
  "d59b25753e6e310cbad655a7176ffe29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinGaussParams>)))
  "Returns full string definition for message of type '<LinGaussParams>"
  (cl:format cl:nil "# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinGaussParams)))
  "Returns full string definition for message of type 'LinGaussParams"
  (cl:format cl:nil "# Time-varying Linear Gaussian controller~%uint32 dX~%uint32 dU~%float64[] K_t  # Should be T x Du x Dx~%float64[] k_t  # Should by T x Du~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinGaussParams>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'K_t) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'k_t) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinGaussParams>))
  "Converts a ROS message object to a list"
  (cl:list 'LinGaussParams
    (cl:cons ':dX (dX msg))
    (cl:cons ':dU (dU msg))
    (cl:cons ':K_t (K_t msg))
    (cl:cons ':k_t (k_t msg))
))
