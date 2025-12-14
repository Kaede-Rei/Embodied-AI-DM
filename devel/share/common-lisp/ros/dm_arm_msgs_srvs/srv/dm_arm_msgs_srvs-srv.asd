
(cl:in-package :asdf)

(defsystem "dm_arm_msgs_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "dm_arm_cmd" :depends-on ("_package_dm_arm_cmd"))
    (:file "_package_dm_arm_cmd" :depends-on ("_package"))
  ))