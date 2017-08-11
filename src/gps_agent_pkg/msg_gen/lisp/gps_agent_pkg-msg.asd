
(cl:in-package :asdf)

(defsystem "gps_agent_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TfParams" :depends-on ("_package_TfParams"))
    (:file "_package_TfParams" :depends-on ("_package"))
    (:file "TfObsData" :depends-on ("_package_TfObsData"))
    (:file "_package_TfObsData" :depends-on ("_package"))
    (:file "ControllerParams" :depends-on ("_package_ControllerParams"))
    (:file "_package_ControllerParams" :depends-on ("_package"))
    (:file "RelaxCommand" :depends-on ("_package_RelaxCommand"))
    (:file "_package_RelaxCommand" :depends-on ("_package"))
    (:file "TrialCommand" :depends-on ("_package_TrialCommand"))
    (:file "_package_TrialCommand" :depends-on ("_package"))
    (:file "DataType" :depends-on ("_package_DataType"))
    (:file "_package_DataType" :depends-on ("_package"))
    (:file "ProxyParams" :depends-on ("_package_ProxyParams"))
    (:file "_package_ProxyParams" :depends-on ("_package"))
    (:file "DataRequest" :depends-on ("_package_DataRequest"))
    (:file "_package_DataRequest" :depends-on ("_package"))
    (:file "PositionCommand" :depends-on ("_package_PositionCommand"))
    (:file "_package_PositionCommand" :depends-on ("_package"))
    (:file "CaffeParams" :depends-on ("_package_CaffeParams"))
    (:file "_package_CaffeParams" :depends-on ("_package"))
    (:file "TfActionCommand" :depends-on ("_package_TfActionCommand"))
    (:file "_package_TfActionCommand" :depends-on ("_package"))
    (:file "SampleResult" :depends-on ("_package_SampleResult"))
    (:file "_package_SampleResult" :depends-on ("_package"))
    (:file "LinGaussParams" :depends-on ("_package_LinGaussParams"))
    (:file "_package_LinGaussParams" :depends-on ("_package"))
  ))