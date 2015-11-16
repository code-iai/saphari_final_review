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
          (let* ((target-object
                   (nth (random (length object-desigs)) object-desigs))
                 (target-location
                   (location-designator `((:a :location)
                                          (:in :sorting-basket)
                                          (:slot-id :middle-slot)
                                          (:target-obj ,target-object)))))
                                          ;; ;; TODO: move pose inference somewhere else
                                          ;; (:pose ,(make-msg
                                          ;;          "geometry_msgs/PoseStamped"
                                          ;;          (:frame_id :header) "sorting_basket"
                                          ;;          (:x :position :pose) 0.25
                                          ;;          (:w :orientation :pose) 1.0))))))
            (let ((updated-target-object (grasp-object demo-handle target-object)))
              (place-object demo-handle updated-target-object target-location))))))))

(defparameter *dh* nil)

(defun bringup-scripting-environment ()
  (start-ros-node "cram")
  (setf *dh* (make-demo-handle)))
