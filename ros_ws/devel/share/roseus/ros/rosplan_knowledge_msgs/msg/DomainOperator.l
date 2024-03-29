;; Auto-generated. Do not edit!


(when (boundp 'rosplan_knowledge_msgs::DomainOperator)
  (if (not (find-package "ROSPLAN_KNOWLEDGE_MSGS"))
    (make-package "ROSPLAN_KNOWLEDGE_MSGS"))
  (shadow 'DomainOperator (find-package "ROSPLAN_KNOWLEDGE_MSGS")))
(unless (find-package "ROSPLAN_KNOWLEDGE_MSGS::DOMAINOPERATOR")
  (make-package "ROSPLAN_KNOWLEDGE_MSGS::DOMAINOPERATOR"))

(in-package "ROS")
;;//! \htmlinclude DomainOperator.msg.html


(defclass rosplan_knowledge_msgs::DomainOperator
  :super ros::object
  :slots (_formula _at_start_add_effects _at_start_del_effects _at_end_add_effects _at_end_del_effects _at_start_assign_effects _at_end_assign_effects _probabilistic_effects _at_start_simple_condition _over_all_simple_condition _at_end_simple_condition _at_start_neg_condition _over_all_neg_condition _at_end_neg_condition _at_start_comparison _at_end_comparison _over_all_comparison ))

(defmethod rosplan_knowledge_msgs::DomainOperator
  (:init
   (&key
    ((:formula __formula) (instance rosplan_knowledge_msgs::DomainFormula :init))
    ((:at_start_add_effects __at_start_add_effects) ())
    ((:at_start_del_effects __at_start_del_effects) ())
    ((:at_end_add_effects __at_end_add_effects) ())
    ((:at_end_del_effects __at_end_del_effects) ())
    ((:at_start_assign_effects __at_start_assign_effects) ())
    ((:at_end_assign_effects __at_end_assign_effects) ())
    ((:probabilistic_effects __probabilistic_effects) ())
    ((:at_start_simple_condition __at_start_simple_condition) ())
    ((:over_all_simple_condition __over_all_simple_condition) ())
    ((:at_end_simple_condition __at_end_simple_condition) ())
    ((:at_start_neg_condition __at_start_neg_condition) ())
    ((:over_all_neg_condition __over_all_neg_condition) ())
    ((:at_end_neg_condition __at_end_neg_condition) ())
    ((:at_start_comparison __at_start_comparison) ())
    ((:at_end_comparison __at_end_comparison) ())
    ((:over_all_comparison __over_all_comparison) ())
    )
   (send-super :init)
   (setq _formula __formula)
   (setq _at_start_add_effects __at_start_add_effects)
   (setq _at_start_del_effects __at_start_del_effects)
   (setq _at_end_add_effects __at_end_add_effects)
   (setq _at_end_del_effects __at_end_del_effects)
   (setq _at_start_assign_effects __at_start_assign_effects)
   (setq _at_end_assign_effects __at_end_assign_effects)
   (setq _probabilistic_effects __probabilistic_effects)
   (setq _at_start_simple_condition __at_start_simple_condition)
   (setq _over_all_simple_condition __over_all_simple_condition)
   (setq _at_end_simple_condition __at_end_simple_condition)
   (setq _at_start_neg_condition __at_start_neg_condition)
   (setq _over_all_neg_condition __over_all_neg_condition)
   (setq _at_end_neg_condition __at_end_neg_condition)
   (setq _at_start_comparison __at_start_comparison)
   (setq _at_end_comparison __at_end_comparison)
   (setq _over_all_comparison __over_all_comparison)
   self)
  (:formula
   (&rest __formula)
   (if (keywordp (car __formula))
       (send* _formula __formula)
     (progn
       (if __formula (setq _formula (car __formula)))
       _formula)))
  (:at_start_add_effects
   (&rest __at_start_add_effects)
   (if (keywordp (car __at_start_add_effects))
       (send* _at_start_add_effects __at_start_add_effects)
     (progn
       (if __at_start_add_effects (setq _at_start_add_effects (car __at_start_add_effects)))
       _at_start_add_effects)))
  (:at_start_del_effects
   (&rest __at_start_del_effects)
   (if (keywordp (car __at_start_del_effects))
       (send* _at_start_del_effects __at_start_del_effects)
     (progn
       (if __at_start_del_effects (setq _at_start_del_effects (car __at_start_del_effects)))
       _at_start_del_effects)))
  (:at_end_add_effects
   (&rest __at_end_add_effects)
   (if (keywordp (car __at_end_add_effects))
       (send* _at_end_add_effects __at_end_add_effects)
     (progn
       (if __at_end_add_effects (setq _at_end_add_effects (car __at_end_add_effects)))
       _at_end_add_effects)))
  (:at_end_del_effects
   (&rest __at_end_del_effects)
   (if (keywordp (car __at_end_del_effects))
       (send* _at_end_del_effects __at_end_del_effects)
     (progn
       (if __at_end_del_effects (setq _at_end_del_effects (car __at_end_del_effects)))
       _at_end_del_effects)))
  (:at_start_assign_effects
   (&rest __at_start_assign_effects)
   (if (keywordp (car __at_start_assign_effects))
       (send* _at_start_assign_effects __at_start_assign_effects)
     (progn
       (if __at_start_assign_effects (setq _at_start_assign_effects (car __at_start_assign_effects)))
       _at_start_assign_effects)))
  (:at_end_assign_effects
   (&rest __at_end_assign_effects)
   (if (keywordp (car __at_end_assign_effects))
       (send* _at_end_assign_effects __at_end_assign_effects)
     (progn
       (if __at_end_assign_effects (setq _at_end_assign_effects (car __at_end_assign_effects)))
       _at_end_assign_effects)))
  (:probabilistic_effects
   (&rest __probabilistic_effects)
   (if (keywordp (car __probabilistic_effects))
       (send* _probabilistic_effects __probabilistic_effects)
     (progn
       (if __probabilistic_effects (setq _probabilistic_effects (car __probabilistic_effects)))
       _probabilistic_effects)))
  (:at_start_simple_condition
   (&rest __at_start_simple_condition)
   (if (keywordp (car __at_start_simple_condition))
       (send* _at_start_simple_condition __at_start_simple_condition)
     (progn
       (if __at_start_simple_condition (setq _at_start_simple_condition (car __at_start_simple_condition)))
       _at_start_simple_condition)))
  (:over_all_simple_condition
   (&rest __over_all_simple_condition)
   (if (keywordp (car __over_all_simple_condition))
       (send* _over_all_simple_condition __over_all_simple_condition)
     (progn
       (if __over_all_simple_condition (setq _over_all_simple_condition (car __over_all_simple_condition)))
       _over_all_simple_condition)))
  (:at_end_simple_condition
   (&rest __at_end_simple_condition)
   (if (keywordp (car __at_end_simple_condition))
       (send* _at_end_simple_condition __at_end_simple_condition)
     (progn
       (if __at_end_simple_condition (setq _at_end_simple_condition (car __at_end_simple_condition)))
       _at_end_simple_condition)))
  (:at_start_neg_condition
   (&rest __at_start_neg_condition)
   (if (keywordp (car __at_start_neg_condition))
       (send* _at_start_neg_condition __at_start_neg_condition)
     (progn
       (if __at_start_neg_condition (setq _at_start_neg_condition (car __at_start_neg_condition)))
       _at_start_neg_condition)))
  (:over_all_neg_condition
   (&rest __over_all_neg_condition)
   (if (keywordp (car __over_all_neg_condition))
       (send* _over_all_neg_condition __over_all_neg_condition)
     (progn
       (if __over_all_neg_condition (setq _over_all_neg_condition (car __over_all_neg_condition)))
       _over_all_neg_condition)))
  (:at_end_neg_condition
   (&rest __at_end_neg_condition)
   (if (keywordp (car __at_end_neg_condition))
       (send* _at_end_neg_condition __at_end_neg_condition)
     (progn
       (if __at_end_neg_condition (setq _at_end_neg_condition (car __at_end_neg_condition)))
       _at_end_neg_condition)))
  (:at_start_comparison
   (&rest __at_start_comparison)
   (if (keywordp (car __at_start_comparison))
       (send* _at_start_comparison __at_start_comparison)
     (progn
       (if __at_start_comparison (setq _at_start_comparison (car __at_start_comparison)))
       _at_start_comparison)))
  (:at_end_comparison
   (&rest __at_end_comparison)
   (if (keywordp (car __at_end_comparison))
       (send* _at_end_comparison __at_end_comparison)
     (progn
       (if __at_end_comparison (setq _at_end_comparison (car __at_end_comparison)))
       _at_end_comparison)))
  (:over_all_comparison
   (&rest __over_all_comparison)
   (if (keywordp (car __over_all_comparison))
       (send* _over_all_comparison __over_all_comparison)
     (progn
       (if __over_all_comparison (setq _over_all_comparison (car __over_all_comparison)))
       _over_all_comparison)))
  (:serialization-length
   ()
   (+
    ;; rosplan_knowledge_msgs/DomainFormula _formula
    (send _formula :serialization-length)
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_add_effects
    (apply #'+ (send-all _at_start_add_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_del_effects
    (apply #'+ (send-all _at_start_del_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_add_effects
    (apply #'+ (send-all _at_end_add_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_del_effects
    (apply #'+ (send-all _at_end_del_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainAssignment[] _at_start_assign_effects
    (apply #'+ (send-all _at_start_assign_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainAssignment[] _at_end_assign_effects
    (apply #'+ (send-all _at_end_assign_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/ProbabilisticEffect[] _probabilistic_effects
    (apply #'+ (send-all _probabilistic_effects :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_simple_condition
    (apply #'+ (send-all _at_start_simple_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_simple_condition
    (apply #'+ (send-all _over_all_simple_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_simple_condition
    (apply #'+ (send-all _at_end_simple_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_neg_condition
    (apply #'+ (send-all _at_start_neg_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_neg_condition
    (apply #'+ (send-all _over_all_neg_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_neg_condition
    (apply #'+ (send-all _at_end_neg_condition :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainInequality[] _at_start_comparison
    (apply #'+ (send-all _at_start_comparison :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainInequality[] _at_end_comparison
    (apply #'+ (send-all _at_end_comparison :serialization-length)) 4
    ;; rosplan_knowledge_msgs/DomainInequality[] _over_all_comparison
    (apply #'+ (send-all _over_all_comparison :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; rosplan_knowledge_msgs/DomainFormula _formula
       (send _formula :serialize s)
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_add_effects
     (write-long (length _at_start_add_effects) s)
     (dolist (elem _at_start_add_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_del_effects
     (write-long (length _at_start_del_effects) s)
     (dolist (elem _at_start_del_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_add_effects
     (write-long (length _at_end_add_effects) s)
     (dolist (elem _at_end_add_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_del_effects
     (write-long (length _at_end_del_effects) s)
     (dolist (elem _at_end_del_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainAssignment[] _at_start_assign_effects
     (write-long (length _at_start_assign_effects) s)
     (dolist (elem _at_start_assign_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainAssignment[] _at_end_assign_effects
     (write-long (length _at_end_assign_effects) s)
     (dolist (elem _at_end_assign_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/ProbabilisticEffect[] _probabilistic_effects
     (write-long (length _probabilistic_effects) s)
     (dolist (elem _probabilistic_effects)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_simple_condition
     (write-long (length _at_start_simple_condition) s)
     (dolist (elem _at_start_simple_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_simple_condition
     (write-long (length _over_all_simple_condition) s)
     (dolist (elem _over_all_simple_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_simple_condition
     (write-long (length _at_end_simple_condition) s)
     (dolist (elem _at_end_simple_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_neg_condition
     (write-long (length _at_start_neg_condition) s)
     (dolist (elem _at_start_neg_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_neg_condition
     (write-long (length _over_all_neg_condition) s)
     (dolist (elem _over_all_neg_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_neg_condition
     (write-long (length _at_end_neg_condition) s)
     (dolist (elem _at_end_neg_condition)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainInequality[] _at_start_comparison
     (write-long (length _at_start_comparison) s)
     (dolist (elem _at_start_comparison)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainInequality[] _at_end_comparison
     (write-long (length _at_end_comparison) s)
     (dolist (elem _at_end_comparison)
       (send elem :serialize s)
       )
     ;; rosplan_knowledge_msgs/DomainInequality[] _over_all_comparison
     (write-long (length _over_all_comparison) s)
     (dolist (elem _over_all_comparison)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; rosplan_knowledge_msgs/DomainFormula _formula
     (send _formula :deserialize buf ptr-) (incf ptr- (send _formula :serialization-length))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_add_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_add_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_start_add_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_del_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_del_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_start_del_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_add_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_add_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_end_add_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_del_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_del_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_end_del_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainAssignment[] _at_start_assign_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_assign_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainAssignment :init) r)) r))
     (dolist (elem- _at_start_assign_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainAssignment[] _at_end_assign_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_assign_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainAssignment :init) r)) r))
     (dolist (elem- _at_end_assign_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/ProbabilisticEffect[] _probabilistic_effects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _probabilistic_effects (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::ProbabilisticEffect :init) r)) r))
     (dolist (elem- _probabilistic_effects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_simple_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_simple_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_start_simple_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_simple_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _over_all_simple_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _over_all_simple_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_simple_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_simple_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_end_simple_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_start_neg_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_neg_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_start_neg_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _over_all_neg_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _over_all_neg_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _over_all_neg_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainFormula[] _at_end_neg_condition
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_neg_condition (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainFormula :init) r)) r))
     (dolist (elem- _at_end_neg_condition)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainInequality[] _at_start_comparison
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_start_comparison (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainInequality :init) r)) r))
     (dolist (elem- _at_start_comparison)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainInequality[] _at_end_comparison
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _at_end_comparison (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainInequality :init) r)) r))
     (dolist (elem- _at_end_comparison)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; rosplan_knowledge_msgs/DomainInequality[] _over_all_comparison
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _over_all_comparison (let (r) (dotimes (i n) (push (instance rosplan_knowledge_msgs::DomainInequality :init) r)) r))
     (dolist (elem- _over_all_comparison)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rosplan_knowledge_msgs::DomainOperator :md5sum-) "169396bd4c8278ca06550ef5a062c243")
(setf (get rosplan_knowledge_msgs::DomainOperator :datatype-) "rosplan_knowledge_msgs/DomainOperator")
(setf (get rosplan_knowledge_msgs::DomainOperator :definition-)
      "# A message used to represent an ungrounded operator in the domain.

# (1) name and parameters
rosplan_knowledge_msgs/DomainFormula formula

# (2) duration constraint


# (3) effect lists
rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
rosplan_knowledge_msgs/DomainAssignment[] at_start_assign_effects
rosplan_knowledge_msgs/DomainAssignment[] at_end_assign_effects
rosplan_knowledge_msgs/ProbabilisticEffect[] probabilistic_effects

# (4) conditions
rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_start_neg_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_neg_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_neg_condition
rosplan_knowledge_msgs/DomainInequality[] at_start_comparison
rosplan_knowledge_msgs/DomainInequality[] at_end_comparison
rosplan_knowledge_msgs/DomainInequality[] over_all_comparison

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

================================================================================
MSG: rosplan_knowledge_msgs/DomainAssignment
# A message used to store the numeric effects of an action
# Can be grounded or ungrounded

uint8 ASSIGN   = 0
uint8 INCREASE  = 1
uint8 DECREASE = 2
uint8 SCALE_UP = 3
uint8 SCALE_DOWN = 4
uint8 ASSIGN_CTS = 5

uint8 assign_type

rosplan_knowledge_msgs/DomainFormula LHS
rosplan_knowledge_msgs/ExprComposite RHS

bool grounded

================================================================================
MSG: rosplan_knowledge_msgs/ExprComposite
# A message used to represent a numerical expression; composite type (2/2)
# stores an array of ExprBase as prefix notation

# components
ExprBase[] tokens

================================================================================
MSG: rosplan_knowledge_msgs/ExprBase
# A message used to represent a numerical expression; base types (1/2)

# expression types
uint8 CONSTANT = 0
uint8 FUNCTION = 1
uint8 OPERATOR = 2
uint8 SPECIAL  = 3

# operators
uint8 ADD    = 0
uint8 SUB    = 1
uint8 MUL    = 2
uint8 DIV    = 3
uint8 UMINUS = 4

# special types
uint8 HASHT      = 0
uint8 TOTAL_TIME = 1
uint8 DURATION   = 2

# expression base type
uint8 expr_type

# constant value
float64 constant

# function
rosplan_knowledge_msgs/DomainFormula function

# operator
uint8 op

# special
uint8 special_type

================================================================================
MSG: rosplan_knowledge_msgs/ProbabilisticEffect
# Message used to represent a Probabilistic effect 
rosplan_knowledge_msgs/ExprComposite   probability
rosplan_knowledge_msgs/DomainFormula[] add_effects # Add effects
rosplan_knowledge_msgs/DomainFormula[] del_effects # Del effects
rosplan_knowledge_msgs/DomainAssignment[] assign_effects # Assign effects, mainly used for RDDL Discrete distribution
================================================================================
MSG: rosplan_knowledge_msgs/DomainInequality
# A message used to store the numeric effects of an action
# Can be grounded or ungrounded

uint8 GREATER   = 0
uint8 GREATEREQ = 1
uint8 LESS      = 2
uint8 LESSEQ    = 3
uint8 EQUALS    = 4

uint8 comparison_type

rosplan_knowledge_msgs/ExprComposite LHS
rosplan_knowledge_msgs/ExprComposite RHS

bool grounded

")



(provide :rosplan_knowledge_msgs/DomainOperator "169396bd4c8278ca06550ef5a062c243")


