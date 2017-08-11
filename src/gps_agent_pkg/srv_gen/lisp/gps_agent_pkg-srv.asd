
(cl:in-package :asdf)

(defsystem "gps_agent_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ProxyControl" :depends-on ("_package_ProxyControl"))
    (:file "_package_ProxyControl" :depends-on ("_package"))
  ))