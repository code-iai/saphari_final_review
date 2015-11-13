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

(in-package :saphari-task-executive)

(defun make-demo-handle (&optional (sim-p t))
  (list
   :json-prolog nil
   :sim-p sim-p
   :tool-perception (ros-interface-tool-perception)
   :beasty (make-and-init-beasty-handle sim-p)
   :wsg50 (make-wsg-handle)
   :tf-broadcaster (advertise "/tf" "tf2_msgs/TFMessage")
   :tf-listener (make-instance 'cl-tf2:buffer-client)
   :marker-pub (advertise "visualization_marker_array" "visualization_msgs/MarkerArray")
   ))

(defun main ()
  (with-ros-node ("cram")
    (let ((demo-handle (make-demo-handle)))
      (cpl:top-level
        ;; TODO: loop
        ;; TODO: human reactivity
        (lookat-pickup-zone demo-handle)
        (alexandria:when-let ((object-desigs (trigger-tool-perception demo-handle)))
          ;; TODO: infer target-object and target-location
          (grasp-object demo-handle (nth (random (length object-desigs)) object-desigs))
          (move-above-target-zone demo-handle)
          (place-object demo-handle nil nil)
      )))))

(defun bringup-scripting-environment ()
  (start-ros-node "cram")
  (setf *dh* (make-demo-handle)))
;;;
;;; BELOW: OLD STATE-MACHINE INTERFACE. MAYBE USEFUL, LATER.
;;; 

;; (defparameter *executive-state* nil)

;; (defun main()
;;   (setf *executive-state* nil)
;;   (with-ros-node ("saphari_task_executive")
;;     (when (init)
;;       (loop-at-most-every 1 (publish-state)))))

;; ;;;
;; ;;; FUNCTIONS USED IN MAIN
;; ;;;

;; (defun init ()
;;   (ros-info :saphari-task-executive "Initializing...")
;;   (if (and
;;        (register-service-fn "~request_state" #'request-state 'saphari_task_executive-srv:requeststate)
;;        (advertise "~state" "saphari_task_executive/State"))
;;     ;; TODO: advertise service
;;       (progn
;;         (ros-info :saphari-task-executive "Init successful.")
;;         (setf *executive-state* :initialized))
;;       (ros-info :saphari-task-executive "Init failed.")))

;; (defun publish-state ()
;;   (publish
;;    "~state"
;;    (make-msg
;;     "saphari_task_executive/State"
;;     :state (symbol-code 'saphari_task_executive-msg:state *executive-state*))))

;; (def-service-callback (request-state saphari_task_executive-srv:requeststate) (state)
;;   (ros-info :saphari-task-executive "~%Requested to change to state: ~a~%" state)
;;   (make-response :success t :status_message "All fine.")
;;   )

;; ;;;
;; ;;; IMPLEMENTATION OF HIGH-LEVEL SERVICE INTERFACE
;; ;;;

;; (defun configure ()
;;   (ros-info :saphari-task-executive "Configuring...")
;;   ;; TODO: implement me
;;   (setf *executive-state* :stopped)
;;   (ros-info :saphari-task-executive "Configure successful.")
;;   t)

;; (defun cleanup ()
;;   (ros-info :saphari-task-executive "Cleaning up...")
;;   ;; TODO: implement me
;;   (setf *executive-state* :initialized)
;;   (ros-info :saphari-task-executive "Cleanup successful.")
;;   t)

;; (defun start ()
;;   (ros-info :saphari-task-executive "Starting...")
;;   ;; TODO: implement me
;;   (setf *executive-state* :running)
;;   (ros-info :saphari-task-executive "Start successful.")
;;   t)

;; (defun stop ()
;;   (ros-info :saphari-task-executive "Starting...")
;;   ;; TODO: implement me
;;   (setf *executive-state* :running)
;;   (ros-info :saphari-task-executive "Start successful.")
;;   t)
