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

(defun infer-motion-goal (desig)
  "Returns the motion goal described by 'desig'. If no matching motion
 goal exists. Throws a CRAM failure if no matching motion goal exists."
  (or
   (infer-lookat-motion-goal desig)
   (infer-pre-place-motion-goal desig)
   (cpl:fail "Could not resolve: ~a" desig)))

(defun infer-lookat-motion-goal (desig)
  "Tries to match 'desig' to its internal description of a goals
 to look at the pickup-zone, and returns the corresponding motion
 goal if the match was successful. If no match is found, it returns
 NIL."
  ;;;
  ;;; NOTE TO MYSELF: this is what I actually want to have
  ;;;
  ;;; (designator-bind ((:an action)
  ;;;                   (:to :see)
  ;;;                   (:obj ((:an :object)
  ;;;                          (:type :pickup-zone)))
  ;;;                   (:distance distance)
  ;;;                   &rest rest)) desig
  ;;;   (declare (ignore rest))
  ;;;   (lookat-pickup-config distance))
  ;;;
  ;;; Semantics:
  ;;; * only looks at descriptions, also for nested designators
  ;;; * like when-let, executes its body only when bindings were successful
  ;;; * binds non-keyword symbols as local variables in its scope
  ;;; * binds any un-matched parts of the description to local variable 'rest'
  ;;; * if &rest is not specified, it will fail in case of un-matched parts in desig
  ;;;
  (and
   (desig-prop-value-p desig :an :action)
   (desig-prop-value-p desig :to :see)
   (desig-descr-equal
    (desig-prop-value desig :obj)
    (object-designator '((:an :object)(:type :pickup-zone))))
   (alexandria:when-let ((distance (desig-prop-value desig :distance))
                         (sim-p (desig-prop-value desig :sim)))
     (roslisp-beasty:make-default-joint-goal
      (lookat-pickup-config distance) sim-p))))

(defun infer-pre-place-motion-goal (desig)
  (and
   (desig-prop-value-p desig :an :action)
   (desig-prop-value-p desig :to :move)
   (alexandria:when-let ((sim-p (desig-prop-value desig :sim))
                         (loc (desig-prop-value desig :at)))
     (and
      (desig-prop-value-p loc :a :location)
      (alexandria:when-let ((obj (desig-prop-value loc :above)))
        (and
         (desig-prop-value-p obj :an :object)
         (desig-prop-value-p obj :type :surgical-basket))))
     (roslisp-beasty:make-default-joint-goal
      ;; TODO: move this into a default function
      (list
       0.5003623962402344
       0.8569384217262268
       0.08925693482160568
       -1.1276057958602905
       -0.0697876513004303
       1.164320468902588
       0.5868684649467468)
      sim-p))))
                          

(defun infer-gripper-goal (desig)
  (or
   (infer-gripper-grasp-goal desig)
   (infer-gripper-place-goal desig)
   (cpl:fail "Could not resolve: ~a" desig)))

(defun infer-gripper-grasp-goal (desig)
  (and
   (desig-prop-value-p desig :an :action)
   (desig-prop-value-p desig :to :grasp)
   ;; TODO: more about objects?
   (list
    cram-wsg50:*wsg50-closed-width*
    cram-wsg50:*default-speed*
    cram-wsg50:*default-force*)))

(defun infer-gripper-place-goal (desig)
  (and
   (desig-prop-value-p desig :an :action)
   (desig-prop-value-p desig :to :place)
   ;; TODO: more about objects?
   (list
    cram-wsg50:*wsg50-open-width*
    cram-wsg50:*default-speed*
    cram-wsg50:*default-force*)))

(defun infer-object-pose (desig)
  (and
   (desig-prop-value-p desig :an :object)
   (alexandria:when-let ((loc-desig (desig-prop-value desig :at)))
     (desig-prop-value loc-desig :pose))))

(defun infer-object-transform (desig)
  (alexandria:when-let ((type-keyword (desig-prop-value desig :type))
                        (pose-stamped (infer-object-pose desig)))
    (pose-stamped->transform-stamped pose-stamped (symbol-name type-keyword))))
