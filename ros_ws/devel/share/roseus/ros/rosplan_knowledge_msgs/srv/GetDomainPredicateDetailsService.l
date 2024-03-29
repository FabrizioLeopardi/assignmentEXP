;; Auto-generated. Do not edit!


(when (boundp 'rosplan_knowledge_msgs::GetDomainPredicateDetailsService)
  (if (not (find-package "ROSPLAN_KNOWLEDGE_MSGS"))
    (make-package "ROSPLAN_KNOWLEDGE_MSGS"))
  (shadow 'GetDomainPredicateDetailsService (find-package "ROSPLAN_KNOWLEDGE_MSGS")))
(unless (find-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICE")
  (make-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICE"))
(unless (find-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICEREQUEST")
  (make-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICEREQUEST"))
(unless (find-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICERESPONSE")
  (make-package "ROSPLAN_KNOWLEDGE_MSGS::GETDOMAINPREDICATEDETAILSSERVICERESPONSE"))

(in-package "ROS")





(defclass rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest
  :super ros::object
  :slots (_name ))

(defmethod rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse
  :super ros::object
  :slots (_predicate _is_sensed ))

(defmethod rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse
  (:init
   (&key
    ((:predicate __predicate) (instance rosplan_knowledge_msgs::DomainFormula :init))
    ((:is_sensed __is_sensed) nil)
    )
   (send-super :init)
   (setq _predicate __predicate)
   (setq _is_sensed __is_sensed)
   self)
  (:predicate
   (&rest __predicate)
   (if (keywordp (car __predicate))
       (send* _predicate __predicate)
     (progn
       (if __predicate (setq _predicate (car __predicate)))
       _predicate)))
  (:is_sensed
   (&optional (__is_sensed :null))
   (if (not (eq __is_sensed :null)) (setq _is_sensed __is_sensed)) _is_sensed)
  (:serialization-length
   ()
   (+
    ;; rosplan_knowledge_msgs/DomainFormula _predicate
    (send _predicate :serialization-length)
    ;; bool _is_sensed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; rosplan_knowledge_msgs/DomainFormula _predicate
       (send _predicate :serialize s)
     ;; bool _is_sensed
       (if _is_sensed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; rosplan_knowledge_msgs/DomainFormula _predicate
     (send _predicate :deserialize buf ptr-) (incf ptr- (send _predicate :serialization-length))
   ;; bool _is_sensed
     (setq _is_sensed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rosplan_knowledge_msgs::GetDomainPredicateDetailsService
  :super ros::object
  :slots ())

(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsService :md5sum-) "a802ceff5ba44f69f622d31862a51d40")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsService :datatype-) "rosplan_knowledge_msgs/GetDomainPredicateDetailsService")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsService :request) rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest)
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsService :response) rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse)

(defmethod rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest
  (:response () (instance rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse :init)))

(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest :md5sum-) "a802ceff5ba44f69f622d31862a51d40")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest :datatype-) "rosplan_knowledge_msgs/GetDomainPredicateDetailsServiceRequest")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceRequest :definition-)
      "# Fetching the Domain (1.5/3):
# Get all details of a specific predicate in the domain.
string name
---
rosplan_knowledge_msgs/DomainFormula predicate
bool is_sensed


================================================================================
MSG: rosplan_knowledge_msgs/DomainFormula
# A message used to represent an atomic formula from the domain.
# typed_parameters matches label -> type
string name
diagnostic_msgs/KeyValue[] typed_parameters

================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time
")

(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse :md5sum-) "a802ceff5ba44f69f622d31862a51d40")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse :datatype-) "rosplan_knowledge_msgs/GetDomainPredicateDetailsServiceResponse")
(setf (get rosplan_knowledge_msgs::GetDomainPredicateDetailsServiceResponse :definition-)
      "# Fetching the Domain (1.5/3):
# Get all details of a specific predicate in the domain.
string name
---
rosplan_knowledge_msgs/DomainFormula predicate
bool is_sensed


================================================================================
MSG: rosplan_knowledge_msgs/DomainFormula
# A message used to represent an atomic formula from the domain.
# typed_parameters matches label -> type
string name
diagnostic_msgs/KeyValue[] typed_parameters

================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time
")



(provide :rosplan_knowledge_msgs/GetDomainPredicateDetailsService "a802ceff5ba44f69f622d31862a51d40")


