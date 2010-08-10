
(in-package :asdf)

(defsystem "move_base_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
 :geometry_msgs-msg
 :roslib-msg
)
  :components ((:file "_package")
    (:file "MoveBaseActionGoal" :depends-on ("_package"))
    (:file "_package_MoveBaseActionGoal" :depends-on ("_package"))
    (:file "MoveBaseActionResult" :depends-on ("_package"))
    (:file "_package_MoveBaseActionResult" :depends-on ("_package"))
    (:file "MoveBaseResult" :depends-on ("_package"))
    (:file "_package_MoveBaseResult" :depends-on ("_package"))
    (:file "MoveBaseAction" :depends-on ("_package"))
    (:file "_package_MoveBaseAction" :depends-on ("_package"))
    (:file "MoveBaseFeedback" :depends-on ("_package"))
    (:file "_package_MoveBaseFeedback" :depends-on ("_package"))
    (:file "MoveBaseGoal" :depends-on ("_package"))
    (:file "_package_MoveBaseGoal" :depends-on ("_package"))
    (:file "MoveBaseActionFeedback" :depends-on ("_package"))
    (:file "_package_MoveBaseActionFeedback" :depends-on ("_package"))
    ))