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

(defmacro with-logging ((begin-hook end-hook) &body body)
  `(let ((log-id (funcall ,begin-hook))
         (result t)) ;; Implicit success
     (labels ((log-succeed ()
                (setf result t))
              (log-fail ()
                (setf result nil)))
       (declare (ignorable (function log-succeed)))
       (declare (ignorable (function log-fail)))
       (unwind-protect (progn ,@body)
         (funcall ,end-hook log-id result)))))

(defun knowrob-annotation-prop (desig)
  (etypecase desig
    (desig:action-designator "designator")
    (desig:object-designator "object-acted-on")
    (desig:location-designator "goal-location")))

(defun on-start-perform-action-designator (desig &rest other-desigs)
  (let ((id (beliefstate:start-node
             "PERFORM-ACTION-DESIGNATOR"
             (list
              (list 'beliefstate::description (desig:description desig))
              ;; matching-process-modules to comply with semrec
              (list 'beliefstate::matching-process-modules nil))
             2)))
    (beliefstate:add-designator-to-node desig id)
    (dolist (other-desig other-desigs)
      (beliefstate:add-designator-to-node
       other-desig id :annotation (knowrob-annotation-prop other-desig)))
    id))

(defun on-finish-perform-action-designator (id success)
  (beliefstate:stop-node id :success success))

(defun on-start-grasping (&rest desigs)
  (let ((id (beliefstate:start-node "GRASP-OBJECT")))
    (dolist (desig desigs)
      (beliefstate:add-designator-to-node
       desig id :annotation (knowrob-annotation-prop desig)))
    id))

(defun on-finish-grasping (id success)
  (beliefstate:stop-node id :success success))

(defun on-start-put-down (desig object location)
  (let ((id (beliefstate:start-node "PUT-DOWN-OBJECT")))
    (dolist (designator (list desig object location))
      (beliefstate:add-designator-to-node
       designator id :annotation (knowrob-annotation-prop designator)))
    id))

(defun on-finish-put-down (id success)
  (beliefstate:stop-node id :success success))

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

(cpl:def-cram-function lookat-target-zone (demo-handle)
  ;; TOOD: use with-designators
  (let ((desig (action-designator
                `((:an :action)
                  (:to :see)
                  (:obj ,(object-designator '((:an :object)
                                              (:type :sorting-basket))))
                  (:sim ,(getf demo-handle :sim-p))))))
    (perform-beasty-motion demo-handle desig)))

(cpl:def-cram-function grasp-object (demo-handle object)
  (let ((desig (action-designator
                `((:an :action)
                  (:to :grasp)
                  (:obj ,object)))))
    (with-logging
        ((alexandria:curry #'on-start-grasping desig object)
         #'on-finish-grasping)
      ;; TODO: refactor to be longer because logs look better?
      (open-gripper demo-handle)
      (reach-object demo-handle object)
      (clamp-object demo-handle object)
      (lookat-pickup-zone demo-handle))
    object))

(cpl:def-cram-function open-gripper (demo-handle)
  (let ((desig (action-designator
                `((:an :action)
                  (:to :open)
                  (:body-part :gripper)))))
    ;; TODO: failure handling
    (perform-gripper-motion demo-handle desig)))

(defun clamp-object (demo-handle object)
  ;; TODO: refactor desig to also hold object?
  (let* ((desig (action-designator
                 `((:an :action)
                   (:to :clamp)
                   (:body-part :gripper)))))
    (perform-gripper-motion demo-handle desig object)
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
    
(defun reach-object (demo-handle object)
  (let ((desig (action-designator
                `((:an :action)
                  (:to :reach)
                  (:obj ,object)
                  (:sim ,(getf demo-handle :sim-p))))))
    (perform-beasty-motion demo-handle desig object)))

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
    (with-logging
        ((alexandria:curry #'on-start-put-down put-down-desig object location)
         #'on-finish-put-down)
      (lookat-target-zone demo-handle)
      (perform-beasty-motion demo-handle reach-desig location)
      (perform-gripper-motion demo-handle release-desig object location)
      ;; TODO: move this code into perform-gripper-motion
      (let ((new-object (desig:copy-designator object :new-description `((:at ,location)))))
        (desig:equate object new-object)
        (publish-tool-markers demo-handle nil new-object)
        ;; TODO: turn into action desig to look at slot
        (lookat-target-zone demo-handle)
        new-object))))
             
             
(defun perform-beasty-motion (demo-handle desig &rest other-log-desigs)
  (with-logging
      ((alexandria:curry #'apply #'on-start-perform-action-designator desig other-log-desigs)
       #'on-finish-perform-action-designator)
    ;; TODO: stuff BEASTY params into action-desig
    (roslisp-beasty:move-beasty-and-wait
     (getf demo-handle :beasty)
     (infer-motion-goal demo-handle desig))))

(defun perform-gripper-motion (demo-handle desig &rest other-log-desigs)
  (with-logging
      ((alexandria:curry #'apply #'on-start-perform-action-designator desig other-log-desigs)
       #'on-finish-perform-action-designator)
    (destructuring-bind (width speed force) (infer-gripper-goal desig)
      (let ((new-desig (desig:copy-designator desig :new-description `((:width ,width) (:speed ,speed) (:force ,force)))))
        (cram-wsg50:move-wsg50-and-wait (getf demo-handle :wsg50) width speed force 5 5)
        (desig:equate desig new-desig)
        new-desig))))
