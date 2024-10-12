;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::SetCartesianImpedance)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'SetCartesianImpedance (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::SETCARTESIANIMPEDANCE")
  (make-package "FRANKA_MSGS::SETCARTESIANIMPEDANCE"))
(unless (find-package "FRANKA_MSGS::SETCARTESIANIMPEDANCEREQUEST")
  (make-package "FRANKA_MSGS::SETCARTESIANIMPEDANCEREQUEST"))
(unless (find-package "FRANKA_MSGS::SETCARTESIANIMPEDANCERESPONSE")
  (make-package "FRANKA_MSGS::SETCARTESIANIMPEDANCERESPONSE"))

(in-package "ROS")





(defclass franka_msgs::SetCartesianImpedanceRequest
  :super ros::object
  :slots (_cartesian_stiffness ))

(defmethod franka_msgs::SetCartesianImpedanceRequest
  (:init
   (&key
    ((:cartesian_stiffness __cartesian_stiffness) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _cartesian_stiffness __cartesian_stiffness)
   self)
  (:cartesian_stiffness
   (&optional __cartesian_stiffness)
   (if __cartesian_stiffness (setq _cartesian_stiffness __cartesian_stiffness)) _cartesian_stiffness)
  (:serialization-length
   ()
   (+
    ;; float64[6] _cartesian_stiffness
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _cartesian_stiffness
     (dotimes (i 6)
       (sys::poke (elt _cartesian_stiffness i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _cartesian_stiffness
   (dotimes (i (length _cartesian_stiffness))
     (setf (elt _cartesian_stiffness i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass franka_msgs::SetCartesianImpedanceResponse
  :super ros::object
  :slots (_success _error ))

(defmethod franka_msgs::SetCartesianImpedanceResponse
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

(defclass franka_msgs::SetCartesianImpedance
  :super ros::object
  :slots ())

(setf (get franka_msgs::SetCartesianImpedance :md5sum-) "c648a0b1340a53eac0c535568e9b5c60")
(setf (get franka_msgs::SetCartesianImpedance :datatype-) "franka_msgs/SetCartesianImpedance")
(setf (get franka_msgs::SetCartesianImpedance :request) franka_msgs::SetCartesianImpedanceRequest)
(setf (get franka_msgs::SetCartesianImpedance :response) franka_msgs::SetCartesianImpedanceResponse)

(defmethod franka_msgs::SetCartesianImpedanceRequest
  (:response () (instance franka_msgs::SetCartesianImpedanceResponse :init)))

(setf (get franka_msgs::SetCartesianImpedanceRequest :md5sum-) "c648a0b1340a53eac0c535568e9b5c60")
(setf (get franka_msgs::SetCartesianImpedanceRequest :datatype-) "franka_msgs/SetCartesianImpedanceRequest")
(setf (get franka_msgs::SetCartesianImpedanceRequest :definition-)
      "float64[6] cartesian_stiffness
---
bool success
string error


")

(setf (get franka_msgs::SetCartesianImpedanceResponse :md5sum-) "c648a0b1340a53eac0c535568e9b5c60")
(setf (get franka_msgs::SetCartesianImpedanceResponse :datatype-) "franka_msgs/SetCartesianImpedanceResponse")
(setf (get franka_msgs::SetCartesianImpedanceResponse :definition-)
      "float64[6] cartesian_stiffness
---
bool success
string error


")



(provide :franka_msgs/SetCartesianImpedance "c648a0b1340a53eac0c535568e9b5c60")


