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

(defun pick-and-place-next-object (demo-handle parent-log-id)
  (with-logging
      ((alexandria:curry #'log-start-pick-and-place parent-log-id)
       (alexandria:rcurry #'log-stop-pick-and-place parent-log-id))
    (alexandria:when-let ((empty-slots (infer-empty-slots demo-handle)))
      (alexandria:when-let ((object-desigs (perceive-instruments demo-handle log-id)))
        (multiple-value-bind (target-object target-location)
            (infer-target-object-and-location-desigs object-desigs)
          (let ((updated-target-object (grasp demo-handle log-id target-object)))
            (put-down demo-handle log-id updated-target-object target-location))))
      empty-slots)))

(defun perceive-instruments (demo-handle parent-log-id)
  (let* ((above-location
           (location-designator
            `((:a :location)
              (:above ,(location-designator
                      '((:a :location)
                        (:in :pickup-zone)))))))
         (move
          (action-designator
           `((:an :action) (:to :move)
             (:at ,above-location) (:sim ,(getf demo-handle :sim-p)))))
         (detect-instruments
           (action-designator
            `((:an :action)
              (:to :detect)
              (:obj ,(object-designator '((:an :object) (:type :surgical-instrument))))))))
    (with-logging
        ((alexandria:curry #'log-start-action-designator parent-log-id detect-instruments)
         (alexandria:rcurry #'log-stop-action-designator parent-log-id))
      (perform-beasty-motion demo-handle log-id move above-location)
      (trigger-tool-perception demo-handle log-id detect-instruments))))

(defun grasp (demo-handle parent-log-id object)
  (let* ((grasp
           (action-designator
            `((:an :action) (:to :grasp) (:obj ,object))))
         (open
           (action-designator
            `((:an :action) (:to :open) (:body-part :gripper))))
         (reach
           (action-designator
            `((:an :action) (:to :reach) (:obj ,object)
              (:sim ,(getf demo-handle :sim-p)))))
         (clamp
           (action-designator
            ;; TODO: add object?
            `((:an :action) (:to :clamp) (:body-part :gripper))))
         (above-location
           (location-designator
            `((:a :location)
              (:above ,(location-designator
                      '((:a :location)
                        (:in :pickup-zone)))))))
         (move
           (action-designator
            `((:an :action) (:to :move)
              (:at ,above-location) (:sim ,(getf demo-handle :sim-p))))))
    (with-logging
        ((alexandria:curry #'log-start-grasping parent-log-id grasp object)
         (alexandria:rcurry #'log-stop-grasping parent-log-id))
      ;; TODO: refactor to be longer because logs look better?
      (perform-gripper-motion demo-handle log-id open)
      (perform-beasty-motion demo-handle log-id reach object)
      (perform-gripper-motion demo-handle log-id clamp object)
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
        new-obj-desig)
      (perform-beasty-motion demo-handle log-id move above-location))
    object))

    
(defun put-down (demo-handle parent-log-id object location)
  (let* ((put-down
          (action-designator
           `((:an :action) (:to :put-down)
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p)))))
        (above-location
          (location-designator
           `((:a :location) (:above ,location))))
        (move
          (action-designator
           `((:an :action) (:to :move)
             (:at ,above-location) (:sim ,(getf demo-handle :sim-p)))))
        (reach
          (action-designator
           `((:an :action) (:to :reach)
             ;; TODO: get rid of object here, we can get type-info out of location
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p)))))
        (release
          (action-designator
           `((:an :action) (:to :release)
             (:obj ,object) (:at ,location)
             (:sim ,(getf demo-handle :sim-p))))))
    (with-logging
        ((alexandria:curry #'log-start-put-down parent-log-id put-down object location)
         (alexandria:rcurry #'log-stop-put-down parent-log-id))
      (perform-beasty-motion demo-handle log-id move above-location)
      (perform-beasty-motion demo-handle log-id reach location)
      (perform-gripper-motion demo-handle log-id release object location)
      ;; TODO: move this code into perform-gripper-motion
      (let ((new-object (desig:copy-designator object :new-description `((:at ,location)))))
        (desig:equate object new-object)
        (publish-tool-markers demo-handle nil new-object))
      (perform-beasty-motion demo-handle log-id move above-location))))
             
             
(defun perform-beasty-motion (demo-handle parent-log-id desig &rest other-log-desigs)
  (with-logging
      ((alexandria:curry #'apply #'log-start-action-designator parent-log-id desig other-log-desigs)
       (alexandria:rcurry #'log-stop-action-designator parent-log-id))
    ;; TODO: stuff BEASTY params into action-desig
    (roslisp-beasty:move-beasty-and-wait
     (getf demo-handle :beasty)
     (infer-motion-goal demo-handle desig))))

(defun perform-gripper-motion (demo-handle parent-log-id desig &rest other-log-desigs)
  (with-logging
      ((alexandria:curry #'apply #'log-start-action-designator parent-log-id desig other-log-desigs)
       (alexandria:rcurry #'log-stop-action-designator parent-log-id))
    (destructuring-bind (width speed force) (infer-gripper-goal desig)
      (let ((new-desig (desig:copy-designator desig :new-description `((:width ,width) (:speed ,speed) (:force ,force)))))
        (cram-wsg50:move-wsg50-and-wait (getf demo-handle :wsg50) width speed force 5 5)
        (desig:equate desig new-desig)
        new-desig))))
