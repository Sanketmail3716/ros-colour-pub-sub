
(cl:in-package :asdf)

(defsystem "custom-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "colour" :depends-on ("_package_colour"))
    (:file "_package_colour" :depends-on ("_package"))
  ))