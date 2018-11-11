; Auto-generated. Do not edit!


(cl:in-package unity_simulation_scene-msg)


;//! \htmlinclude carPosition.msg.html

(cl:defclass <carPosition> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass carPosition (<carPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <carPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'carPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unity_simulation_scene-msg:<carPosition> is deprecated: use unity_simulation_scene-msg:carPosition instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <carPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unity_simulation_scene-msg:id-val is deprecated.  Use unity_simulation_scene-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <carPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unity_simulation_scene-msg:position-val is deprecated.  Use unity_simulation_scene-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <carPosition>) ostream)
  "Serializes a message object of type '<carPosition>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <carPosition>) istream)
  "Deserializes a message object of type '<carPosition>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<carPosition>)))
  "Returns string type for a message object of type '<carPosition>"
  "unity_simulation_scene/carPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'carPosition)))
  "Returns string type for a message object of type 'carPosition"
  "unity_simulation_scene/carPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<carPosition>)))
  "Returns md5sum for a message object of type '<carPosition>"
  "5bdcf5493d4c37123fac06af099d8c53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'carPosition)))
  "Returns md5sum for a message object of type 'carPosition"
  "5bdcf5493d4c37123fac06af099d8c53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<carPosition>)))
  "Returns full string definition for message of type '<carPosition>"
  (cl:format cl:nil "string id~%geometry_msgs/Vector3 position~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'carPosition)))
  "Returns full string definition for message of type 'carPosition"
  (cl:format cl:nil "string id~%geometry_msgs/Vector3 position~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <carPosition>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <carPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'carPosition
    (cl:cons ':id (id msg))
    (cl:cons ':position (position msg))
))
