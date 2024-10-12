;; Auto-generated. Do not edit!


(when (boundp 'franka_msgs::SetEEFrame)
  (if (not (find-package "FRANKA_MSGS"))
    (make-package "FRANKA_MSGS"))
  (shadow 'SetEEFrame (find-package "FRANKA_MSGS")))
(unless (find-package "FRANKA_MSGS::SETEEFRAME")
  (make-package "FRANKA_MSGS::SETEEFRAME"))
(unless (find-package "FRANKA_MSGS::SETEEFRAMEREQUEST")
  (make-package "FRANKA_MSGS::SETEEFRAMEREQUEST"))
(unless (find-package "FRANKA_MSGS::SETEEFRAMERESPONSE")
  (make-package "FRANKA_MSGS::SETEEFRAMERESPONSE"))

(in-package "ROS")





(defclass franka_msgs::SetEEFrameRequest
  :super ros::object
  :slots (_NE_T_EE ))

(defmethod franka_msgs::SetEEFrameRequest
  (:init
   (&key
    ((:NE_T_EE __NE_T_EE) (make-array 16 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _NE_T_EE __NE_T_EE)
   self)
  (:NE_T_EE
   (&optional __NE_T_EE)
   (if __NE_T_EE (setq _NE_T_EE __NE_T_EE)) _NE_T_EE)
  (:serialization-length
   ()
   (+
    ;; float64[16] _NE_T_EE
    (* 8    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[16] _NE_T_EE
     (dotimes (i 16)
       (sys::poke (elt _NE_T_EE i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[16] _NE_T_EE
   (dotimes (i (length _NE_T_EE))
     (setf (elt _NE_T_EE i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass franka_msgs::SetEEFrameResponse
  :super ros::object
  :slots (_success _error ))

(defmethod franka_msgs::SetEEFrameResponse
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

(defclass franka_msgs::SetEEFrame
  :super ros::object
  :slots ())

(setf (get franka_msgs::SetEEFrame :md5sum-) "459aea8fa6dbdd34ba4bc52d3eddbacf")
(setf (get franka_msgs::SetEEFrame :datatype-) "franka_msgs/SetEEFrame")
(setf (get franka_msgs::SetEEFrame :request) franka_msgs::SetEEFrameRequest)
(setf (get franka_msgs::SetEEFrame :response) franka_msgs::SetEEFrameResponse)

(defmethod franka_msgs::SetEEFrameRequest
  (:response () (instance franka_msgs::SetEEFrameResponse :init)))

(setf (get franka_msgs::SetEEFrameRequest :md5sum-) "459aea8fa6dbdd34ba4bc52d3eddbacf")
(setf (get franka_msgs::SetEEFrameRequest :datatype-) "franka_msgs/SetEEFrameRequest")
(setf (get franka_msgs::SetEEFrameRequest :definition-)
      "float64[16] NE_T_EE
---
bool success
string error


")

(setf (get franka_msgs::SetEEFrameResponse :md5sum-) "459aea8fa6dbdd34ba4bc52d3eddbacf")
(setf (get franka_msgs::SetEEFrameResponse :datatype-) "franka_msgs/SetEEFrameResponse")
(setf (get franka_msgs::SetEEFrameResponse :definition-)
      "float64[16] NE_T_EE
---
bool success
string error


")



(provide :franka_msgs/SetEEFrame "459aea8fa6dbdd34ba4bc52d3eddbacf")


