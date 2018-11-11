
(cl:in-package :asdf)

(defsystem "unity_communication-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "carPosition" :depends-on ("_package_carPosition"))
    (:file "_package_carPosition" :depends-on ("_package"))
  ))