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

(cpl-impl:def-cram-function perceive-tools (demo-handle &rest desigs)
  (cpl:try-in-order
    (apply #'query-tool-perception demo-handle desigs)
    (cpl:seq
      (lookat-pickup-zone demo-handle)
      (ros-info :saphari-task-executive "Moving arm to look at pickup zone.")
      (trigger-tool-perception demo-handle)
      (apply #'query-tool-perception demo-handle desigs))))

(cpl:def-cram-function lookat-pickup-zone (demo-handle &optional (distance 30))
  ;; TOOD: use with-designators
  (let ((desig (action-designator
                `((:an :action)
                  (:to :see)
                  (:obj ,(object-designator '((:an :object)
                                              (:type :pickup-zone))))
                  (:distance ,distance)
                  (:sim ,(getf demo-handle :sim-p))))))
    (perform-beasty-motion demo-handle desig)))

(cpl:def-cram-function move-above-target-zone (demo-handle)
  ;; TODO: use with-designators
  (let ((desig (action-designator
                `((:an :action)
                  (:to :move)
                  (:sim ,(getf demo-handle :sim-p))
                  (:at ,(location-designator
                         `((:a :location)
                           (:above ,(object-designator
                                     '((:an :object)
                                       (:type :surgical-basket)))))))))))
     (perform-beasty-motion demo-handle desig)))

(cpl:def-cram-function grasp-object (demo-handle object)
  ;; TODO: check nothing in hand?
  ;; TODO: object perceived?
  ;; TODO: failure handling
  (open-gripper demo-handle)
  (reach-object demo-handle object)
  (clamp-object demo-handle object))

(cpl:def-cram-function open-gripper (demo-handle)
  (let ((desig (action-designator
                `((:an :action)
                  (:to :open)
                  (:body-part :gripper)))))
    ;; TODO: failure handling
    (perform-gripper-motion demo-handle desig)))

(cpl:def-cram-function clamp-object (demo-handle object)
  ;; TODO: refactor desig to also hold object?
  (let ((desig (action-designator
                `((:an :action)
                  (:to :close)
                  (:body-part :gripper)))))
    ;; TODO: failure handling
    (perform-gripper-motion demo-handle desig)
    (alexandria:when-let*
        ((obj-in-gripper-pose
          (transform->pose-stamped-msg
           (infer-object-grasping-offset object)
           "gripper_tool_frame"))
         (new-obj-desig
          (desig:copy-designator
           object
           :new-description
           `((:at ,(location-designator
                    `((:in :gripper)
                      (:pose ,obj-in-gripper-pose))))))))
      (desig:equate object new-obj-desig)
      (publish-tool-markers demo-handle t new-obj-desig)
      new-obj-desig)))
    
(cpl:def-cram-function reach-object (demo-handle object)
  (let ((desig (action-designator
                     `((:an :action)
                       (:to :grasp)
                       (:obj ,object)
                       (:sim ,(getf demo-handle :sim-p))))))
    ;; TODO: failure handling
    (perform-beasty-motion demo-handle desig)))

(cpl:def-cram-function place-object (demo-handle object location)
  ;; TODO: verify object in hand?
  ;; TODO: verify location to place this object?
  ;; TODO: failure handling
  (reach-location demo-handle location)
  (release-object demo-handle object))

(cpl:def-cram-function reach-location (demo-handle location)
  (let ((desig
          (action-designator
           `((:an :action)
             (:to :reach)
             (:at ,location)
             (:sim ,(getf demo-handle :sim-p))))))
    ;; TODO: failure handling
    (perform-beasty-motion demo-handle desig)))

(cpl:def-cram-function release-object (demo-handle object)
  (let ((desig
          (action-designator
           `((:an :action)
             (:to :release)
             (:obj ,object)))))
    (perform-gripper-motion demo-handle desig)
    (alexandria:when-let*
        ((obj-in-gripper-pose
          (transform->pose-stamped-msg
           (infer-object-grasping-offset object)
           "gripper_tool_frame"))
         (new-obj-desig
          (desig:copy-designator
           object
           :new-description
           `((:at ,(location-designator
                    `((:pose ,obj-in-gripper-pose))))))))
      (desig:equate object new-obj-desig)
      (publish-tool-markers demo-handle nil new-obj-desig)
      new-obj-desig)))
             
             
;; TODO: turn me into a plan
(defun perform-beasty-motion (demo-handle desig)
  (roslisp-beasty:move-beasty-and-wait
   (getf demo-handle :beasty)
   (infer-motion-goal demo-handle desig)))

(cpl:def-cram-function perform-gripper-motion (demo-handle desig)
  (destructuring-bind (width speed force) (infer-gripper-goal desig)
    (cram-wsg50:move-wsg50-and-wait (getf demo-handle :wsg50) width speed force 5 5)))
