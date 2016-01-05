
(cl:in-package :asdf)

(defsystem "teb_local_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObstacleMsg" :depends-on ("_package_ObstacleMsg"))
    (:file "_package_ObstacleMsg" :depends-on ("_package"))
  ))