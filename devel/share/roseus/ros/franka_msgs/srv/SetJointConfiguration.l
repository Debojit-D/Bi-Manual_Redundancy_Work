;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::SetJointConfiguration)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'SetJointConfiguration (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::SETJOINTCONFIGURATION")
  (make-package "FRANKA_MSGS::SETJOINTCONFIGURATION"))
(unless (find-package "FRANKA_MSGS::SETJOINTCONFIGURATIONREQUEST")
  (make-package "FRANKA_MSGS::SETJOINTCONFIGURATIONREQUEST"))
(unless (find-package "FRANKA_MSGS::SETJOINTCONFIGURATIONRESPONSE")
  (make-package "FRANKA_MSGS::SETJOINTCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass franka_msgs::SetJointConfigurationRequest
  :super ros::object
  :slots (_joint_names _joint_positions ))

(defmethod franka_msgs::SetJointConfigurationRequest
  (:init
   (&key
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:joint_positions __joint_positions) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_names __joint_names)
   (setq _joint_positions __joint_positions)
   self)
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:serialization-length
   ()
   (+
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; float64[] _joint_positions
    (* 8    (length _joint_positions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _joint_positions
     (write-long (length _joint_positions) s)
     (dotimes (i (length _joint_positions))
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _joint_positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass franka_msgs::SetJointConfigurationResponse
  :super ros::object
  :slots (_success _error ))

(defmethod franka_msgs::SetJointConfigurationResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error __error) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _error (string __error))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _error
    4 (length _error)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _error
       (write-long (length _error) s) (princ _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _error
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass franka_msgs::SetJointConfiguration
  :super ros::object
  :slots ())

(setf (get franka_msgs::SetJointConfiguration :md5sum-) "772aebee560c498b58210c453ba3c24c")
(setf (get franka_msgs::SetJointConfiguration :datatype-) "franka_msgs/SetJointConfiguration")
(setf (get franka_msgs::SetJointConfiguration :request) franka_msgs::SetJointConfigurationRequest)
(setf (get franka_msgs::SetJointConfiguration :response) franka_msgs::SetJointConfigurationResponse)

(defmethod franka_msgs::SetJointConfigurationRequest
  (:response () (instance franka_msgs::SetJointConfigurationResponse :init)))

(setf (get franka_msgs::SetJointConfigurationRequest :md5sum-) "772aebee560c498b58210c453ba3c24c")
(setf (get franka_msgs::SetJointConfigurationRequest :datatype-) "franka_msgs/SetJointConfigurationRequest")
(setf (get franka_msgs::SetJointConfigurationRequest :definition-)
      "string[] joint_names
float64[] joint_positions
---
bool success
string error

")

(setf (get franka_msgs::SetJointConfigurationResponse :md5sum-) "772aebee560c498b58210c453ba3c24c")
(setf (get franka_msgs::SetJointConfigurationResponse :datatype-) "franka_msgs/SetJointConfigurationResponse")
(setf (get franka_msgs::SetJointConfigurationResponse :definition-)
      "string[] joint_names
float64[] joint_positions
---
bool success
string error

")



(provide :franka_msgs/SetJointConfiguration "772aebee560c498b58210c453ba3c24c")


