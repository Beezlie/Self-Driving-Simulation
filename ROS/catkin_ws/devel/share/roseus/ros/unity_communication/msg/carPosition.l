;; Auto-generated. Do not edit!


(when (boundp 'unity_communication::carPosition)
  (if (not (find-package "UNITY_COMMUNICATION"))
    (make-package "UNITY_COMMUNICATION"))
  (shadow 'carPosition (find-package "UNITY_COMMUNICATION")))
(unless (find-package "UNITY_COMMUNICATION::CARPOSITION")
  (make-package "UNITY_COMMUNICATION::CARPOSITION"))

(in-package "ROS")
;;//! \htmlinclude carPosition.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass unity_communication::carPosition
  :super ros::object
  :slots (_id _position ))

(defmethod unity_communication::carPosition
  (:init
   (&key
    ((:id __id) "")
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _position __position)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;;
   self)
  )

(setf (get unity_communication::carPosition :md5sum-) "5bdcf5493d4c37123fac06af099d8c53")
(setf (get unity_communication::carPosition :datatype-) "unity_communication/carPosition")
(setf (get unity_communication::carPosition :definition-)
      "string id
geometry_msgs/Vector3 position

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :unity_communication/carPosition "5bdcf5493d4c37123fac06af099d8c53")


