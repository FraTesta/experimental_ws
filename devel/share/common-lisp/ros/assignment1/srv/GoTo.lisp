; Auto-generated. Do not edit!


(cl:in-package assignment1-srv)


;//! \htmlinclude GoTo-request.msg.html

(cl:defclass <GoTo-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass GoTo-request (<GoTo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoTo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoTo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1-srv:<GoTo-request> is deprecated: use assignment1-srv:GoTo-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GoTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-srv:x-val is deprecated.  Use assignment1-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GoTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-srv:y-val is deprecated.  Use assignment1-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoTo-request>) ostream)
  "Serializes a message object of type '<GoTo-request>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoTo-request>) istream)
  "Deserializes a message object of type '<GoTo-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoTo-request>)))
  "Returns string type for a service object of type '<GoTo-request>"
  "assignment1/GoToRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo-request)))
  "Returns string type for a service object of type 'GoTo-request"
  "assignment1/GoToRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoTo-request>)))
  "Returns md5sum for a message object of type '<GoTo-request>"
  "6835bdfd1be90305ffbc4f2fb8805255")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoTo-request)))
  "Returns md5sum for a message object of type 'GoTo-request"
  "6835bdfd1be90305ffbc4f2fb8805255")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoTo-request>)))
  "Returns full string definition for message of type '<GoTo-request>"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoTo-request)))
  "Returns full string definition for message of type 'GoTo-request"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoTo-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoTo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoTo-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude GoTo-response.msg.html

(cl:defclass <GoTo-response> (roslisp-msg-protocol:ros-message)
  ((o
    :reader o
    :initarg :o
    :type cl:integer
    :initform 0))
)

(cl:defclass GoTo-response (<GoTo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoTo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoTo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1-srv:<GoTo-response> is deprecated: use assignment1-srv:GoTo-response instead.")))

(cl:ensure-generic-function 'o-val :lambda-list '(m))
(cl:defmethod o-val ((m <GoTo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-srv:o-val is deprecated.  Use assignment1-srv:o instead.")
  (o m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoTo-response>) ostream)
  "Serializes a message object of type '<GoTo-response>"
  (cl:let* ((signed (cl:slot-value msg 'o)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoTo-response>) istream)
  "Deserializes a message object of type '<GoTo-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'o) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoTo-response>)))
  "Returns string type for a service object of type '<GoTo-response>"
  "assignment1/GoToResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo-response)))
  "Returns string type for a service object of type 'GoTo-response"
  "assignment1/GoToResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoTo-response>)))
  "Returns md5sum for a message object of type '<GoTo-response>"
  "6835bdfd1be90305ffbc4f2fb8805255")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoTo-response)))
  "Returns md5sum for a message object of type 'GoTo-response"
  "6835bdfd1be90305ffbc4f2fb8805255")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoTo-response>)))
  "Returns full string definition for message of type '<GoTo-response>"
  (cl:format cl:nil "int64 o~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoTo-response)))
  "Returns full string definition for message of type 'GoTo-response"
  (cl:format cl:nil "int64 o~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoTo-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoTo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoTo-response
    (cl:cons ':o (o msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoTo)))
  'GoTo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoTo)))
  'GoTo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo)))
  "Returns string type for a service object of type '<GoTo>"
  "assignment1/GoTo")