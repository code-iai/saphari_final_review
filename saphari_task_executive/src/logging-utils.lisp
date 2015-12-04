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

(defmacro with-log-extraction (&body body)
  `(progn
     (beliefstate::init-semrec)
     (beliefstate:enable-logging t)
     (unwind-protect
          ,@body
       (beliefstate:extract-files))))
  
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

(defun log-start-action-designator (parent-log-id desig &rest other-desigs)
  (let ((id (beliefstate:start-node
             "PERFORM-ACTION-DESIGNATOR"
             (list
              (list 'beliefstate::description (desig:description desig))
              ;; matching-process-modules to comply with semrec
              (list 'beliefstate::matching-process-modules nil))
             2
             parent-log-id)))
    (beliefstate:add-designator-to-node desig id)
    (dolist (other-desig other-desigs)
      (beliefstate:add-designator-to-node
       other-desig id :annotation (knowrob-annotation-prop other-desig)))
    id))

(defun log-stop-action-designator (id success parent-log-id)
  (beliefstate:stop-node id :success success :relative-context-id parent-log-id))

(defun log-start-grasping (parent-log-id &rest desigs)
  (let ((id (beliefstate:start-node "GRASP-OBJECT" nil 2 parent-log-id)))
    (dolist (desig desigs)
      (beliefstate:add-designator-to-node
       desig id :annotation (knowrob-annotation-prop desig)))
    id))

(defun log-stop-grasping (id success parent-log-id)
  (beliefstate:stop-node id :success success :relative-context-id parent-log-id))

(defun log-start-put-down (parent-log-id desig object location)
  (let ((id (beliefstate:start-node "PUT-DOWN-OBJECT" nil 2 parent-log-id)))
    (dolist (designator (list desig object location))
      (beliefstate:add-designator-to-node
       designator id :annotation (knowrob-annotation-prop designator)))
    id))

(defun log-stop-put-down (id success parent-log-id)
  (beliefstate:stop-node id :success success :relative-context-id parent-log-id))

(defun log-start-pick-and-place (parent-log-id)
  (beliefstate:start-node "PICK-AND-PLACE" nil 2 parent-log-id))

(defun log-stop-pick-and-place (log-id success parent-log-id)
  (beliefstate:stop-node log-id :success success :relative-context-id parent-log-id))

(defun test (&key a b)
  (format t "~a ~a~%" a b))
