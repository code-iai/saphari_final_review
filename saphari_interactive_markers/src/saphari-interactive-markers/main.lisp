;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :saphari-interactive-markers)

;;;
;;; TESTING UTIL
;;;

(defun test-goal-pose ()
  (make-message
   "geometry_msgs/PoseStamped"
   (:frame_id :header) "gripper_tool_frame"
   (:stamp :header) (ros-time)
   (:w :orientation :pose) 1.0
   (:x :position :pose) 0.1))

;;;
;;; MESSAGE CONVERSIONS
;;;

(defun point-msg->point (msg)
  (with-fields (x y z) msg
    (cl-transforms:make-3d-vector x y z)))

(defun quaternion-msg->quaternion (msg)
  (with-fields (x y z w) msg
    (cl-transforms:make-quaternion x y z w)))
 
(defun pose-msg->pose (msg)
  (with-fields (position orientation) msg
    (cl-transforms:make-pose
     (point-msg->point position)
     (quaternion-msg->quaternion orientation))))

;;;
;;; FUNCTIONS COMPRISING ACTUAL PROGRAM
;;;


(defun tf2-lookup (tf frame-id child-frame-id)
  (handler-case (cl-tf2:transform (cl-tf2:lookup-transform tf frame-id child-frame-id))
    (cl-tf2::tf2-server-error () (progn (sleep 0.1) (tf2-lookup tf frame-id child-frame-id)))))
                                                         
(defun init-ros-components (&optional (sim-p t))
  (let ((tf (make-instance 'cl-tf2:buffer-client))
        (beasty (make-and-init-beasty-handle "beasty" 1 1337 sim-p)))
    (list tf beasty)))

(defun relay-beasty-goal-msg (tf beasty goal-msg &optional (sim-p t))
  (with-fields ((frame-id (frame_id header)) (pose-msg pose)) goal-msg
    (ros-debug :relay-beasty-goal "In: ~a" goal-msg)
    (let* ((arm-transform (tf2-lookup tf "arm_base_link" frame-id))
           (goal-pose
            (cl-transforms:pose->transform
             (cl-transforms:transform-pose arm-transform (pose-msg->pose pose-msg)))))
      (ros-debug :relay-beasty-goal "Arm-transform: ~a" arm-transform)
      (ros-debug :relay-beasty-goal "Out: ~a" goal-pose)
      (move-beasty-and-wait beasty (make-default-cartesian-goal goal-pose sim-p)))))  

;;;
;;; ENTRY POINT FOR SCRIPT
;;;

(defun main (&optional (node-name "beasty_goal_relay"))
  (with-ros-node (node-name :spin t)
    (let ((callback (apply #'alexandria:curry #'relay-beasty-goal-msg (init-ros-components))))
      (subscribe "/saphari_interactive_markers/goal_out" "geometry_msgs/PoseStamped" callback))))     
