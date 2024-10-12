;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::SetJointImpedance)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'SetJointImpedance (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::SETJOINTIMPEDANCE")
  (make-package "FRANKA_MSGS::SETJOINTIMPEDANCE"))
(unless (find-package "FRANKA_MSGS::SETJOINTIMPEDANCEREQUEST")
  (make-package "FRANKA_MSGS::SETJOINTIMPEDANCEREQUEST"))
(unless (find-package "FRANKA_MSGS::SETJOINTIMPEDANCERESPONSE")
  (make-package "FRANKA_MSGS::SETJOINTIMPEDANCERESPONSE"))

(in-package "ROS")





(defclass franka_msgs::SetJointImpedanceRequest
  :super ros::object
  :slots (_joint_stiffness ))

(defmethod franka_msgs::SetJointImpedanceRequest
  (:init
   (&key
    ((:joint_stiffness __joint_stiffness) (make-array 7 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_stiffness __joint_stiffness)
   self)
  (:joint_stiffness
   (&optional __joint_stiffness)
   (if __joint_stiffness (setq _joint_stiffness __joint_stiffness)) _joint_stiffness)
  (:serialization-length
   ()
   (+
    ;; float64[7] _joint_stiffness
    (* 8    7)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[7] _joint_stiffness
     (dotimes (i 7)
       (sys::poke (elt _joint_stiffness i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[7] _joint_stiffness
   (dotimes (i (length _joint_stiffness))
     (setf (elt _joint_stiffness i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass franka_msgs::SetJointImpedanceResponse
  :super ros::object
  :slots (_success _error ))

(defmethod franka_msgs::SetJointImpedanceResponse
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

(defclass franka_msgs::SetJointImpedance
  :super ros::object
  :slots ())

(setf (get franka_msgs::SetJointImpedance :md5sum-) "f11c4defa2a13040512214f9d77dc982")
(setf (get franka_msgs::SetJointImpedance :datatype-) "franka_msgs/SetJointImpedance")
(setf (get franka_msgs::SetJointImpedance :request) franka_msgs::SetJointImpedanceRequest)
(setf (get franka_msgs::SetJointImpedance :response) franka_msgs::SetJointImpedanceResponse)

(defmethod franka_msgs::SetJointImpedanceRequest
  (:response () (instance franka_msgs::SetJointImpedanceResponse :init)))

(setf (get franka_msgs::SetJointImpedanceRequest :md5sum-) "f11c4defa2a13040512214f9d77dc982")
(setf (get franka_msgs::SetJointImpedanceRequest :datatype-) "franka_msgs/SetJointImpedanceRequest")
(setf (get franka_msgs::SetJointImpedanceRequest :definition-)
      "float64[7] joint_stiffness
---
bool success
string error


")

(setf (get franka_msgs::SetJointImpedanceResponse :md5sum-) "f11c4defa2a13040512214f9d77dc982")
(setf (get franka_msgs::SetJointImpedanceResponse :datatype-) "franka_msgs/SetJointImpedanceResponse")
(setf (get franka_msgs::SetJointImpedanceResponse :definition-)
      "float64[7] joint_stiffness
---
bool success
string error


")



(provide :franka_msgs/SetJointImpedance "f11c4defa2a13040512214f9d77dc982")


