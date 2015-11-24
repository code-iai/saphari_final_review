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

(defun on-start-perform-action-designator (desig)
  (declare (type desig:action-designator desig))
  (let ((id (beliefstate:start-node
             "PERFORM-ACTION-DESIGNATOR"
             (list
              (list 'beliefstate::description (desig:description desig))
              ;; matching-process-modules to comply with semrec
              (list 'beliefstate::matching-process-modules nil))
             2)))
    (beliefstate:add-designator-to-node desig id)
    id))

(defun on-finish-perform-action-designator (id)
  (beliefstate:stop-node id))

(defun on-start-grasping (desig)
  (let ((id (beliefstate:start-node "GRASP-OBJECT")))
    (beliefstate:add-designator-to-node desig id :annotation "designator")))

(defun on-finish-grasping (id)
  (beliefstate:stop-node id))

(defun on-start-put-down (desig)
  (let ((id (beliefstate:start-node "PUT-DOWN-OBJECT")))
    (beliefstate:add-designator-to-node desig id :annotation "designator")))

(defun on-finish-put-down (id)
  (beliefstate:stop-node id))

(cpl:def-cram-function lookat-target-zone (demo-handle)
  ;; TOOD: use with-designators
  (let ((desig (action-designator
                `((:an :action)
                  (:to :see)
                  (:obj ,(object-designator '((:an :object)
                                              (:type :sorting-basket))))
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
  (let* ((desig (action-designator
                 `((:an :action)
                   (:to :grasp)
                   (:obj ,object))))
         (logging-id (on-start-grasping desig)))
    (open-gripper demo-handle)
    (reach-object demo-handle object)
    (clamp-object demo-handle object)
    (lookat-pickup-zone demo-handle)
    (on-finish-grasping logging-id)
    object))

(cpl:def-cram-function open-gripper (demo-handle)
  (let ((desig (action-designator
                `((:an :action)
                  (:to :open)
                  (:body-part :gripper)))))
    ;; TODO: failure handling
    (perform-gripper-motion demo-handle desig)))

(cpl:def-cram-function clamp-object (demo-handle object)
  ;; TODO: refactor desig to also hold object?
  (let* ((desig (action-designator
                `((:an :action)
                  (:to :clamp)
                  (:body-part :gripper))))
         (logging-id (on-start-perform-action-designator desig)))
    ;; TODO: failure handling
    (perform-gripper-motion demo-handle desig)
    (on-finish-perform-action-designator logging-id)
    ;; TODO: move this into perform-gripper-motion
    (alexandria:when-let*
        ((obj-in-gripper-pose
          (transform->pose-stamped-msg
           (infer-object-grasping-offset object)
           "gripper_tool_frame"))
         (new-obj-desig
          (desig:copy-designator
           object
           :new-description
           `((:at ,(pose-stamped->loc-desig obj-in-gripper-pose '((:in :gripper))))))))
      (desig:equate object new-obj-desig)
      (publish-tool-markers demo-handle t new-obj-desig)
      new-obj-desig)))
    
(cpl:def-cram-function reach-object (demo-handle object)
  (let* ((desig (action-designator
                     `((:an :action)
                       (:to :reach)
                       (:obj ,object)
                       (:sim ,(getf demo-handle :sim-p)))))
         (logging-id (on-start-perform-action-designator desig)))
    ;; TODO: failure handling
    (perform-beasty-motion demo-handle desig)
    (on-finish-perform-action-designator logging-id)))

(cpl:def-cram-function put-down (demo-handle object location)
  (let ((put-down-desig
          (action-designator
           `((:an :action) (:to :put-down)
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p)))))
        (reach-desig
          (action-designator
           `((:an :action) (:to :reach)
             ;; TODO: get rid of object here, we can get type-info out of location
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p)))))
        (release-desig
          (action-designator
           `((:an :action) (:to :release)
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p))))))
    ;; TODO: failure handling
    ;; TODO: turn into action desig to look at slot
    (let ((logging-id (on-start-put-down put-down-desig)))
      (lookat-target-zone demo-handle)
      (let ((logging-id (on-start-perform-action-designator reach-desig)))
        ;; TODO: failure handling
        (perform-beasty-motion demo-handle reach-desig)
        (on-finish-perform-action-designator logging-id))
      (let ((logging-id (on-start-perform-action-designator release-desig)))
        ;; TODO: failure handling
        (perform-gripper-motion demo-handle release-desig)
        (on-finish-perform-action-designator logging-id))
      ;; TODO: move this code into perform-gripper-motion
      (let ((new-object (desig:copy-designator object :new-description `((:at ,location)))))
        (desig:equate object new-object)
        (publish-tool-markers demo-handle nil new-object)
        ;; TODO: turn into action desig to look at slot
        (lookat-target-zone demo-handle)
        (on-finish-put-down logging-id)
        new-object))
    ;; (alexandria:when-let*
    ;;     ((obj-in-gripper-pose
    ;;       (transform->pose-stamped-msg
    ;;        (infer-object-grasping-offset object)
    ;;        "gripper_tool_frame"))
    ;;      (new-obj-desig
    ;;       (desig:copy-designator
    ;;        object
    ;;        :new-description
    ;;        `((:at ,(pose-stamped->loc-desig obj-in-gripper-pose '((:in :basket))))))))
    ;;   (desig:equate object new-obj-desig)
    ;;   (publish-tool-markers demo-handle nil new-obj-desig)
    ;;   new-obj-desig)
            ))
             
             
;; TODO: turn me into a plan
(cpl:def-cram-function perform-beasty-motion (demo-handle desig)
  (roslisp-beasty:move-beasty-and-wait
   (getf demo-handle :beasty)
   (infer-motion-goal demo-handle desig)))

(cpl:def-cram-function perform-gripper-motion (demo-handle desig)
  (destructuring-bind (width speed force) (infer-gripper-goal desig)
    (let ((new-desig (desig:copy-designator desig :new-description `((:width ,width) (:speed ,speed) (:force ,force)))))
      (cram-wsg50:move-wsg50-and-wait (getf demo-handle :wsg50) width speed force 5 5)
      (desig:equate desig new-desig)
      new-desig)))
