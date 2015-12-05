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

(defun log-track-new-human (user-id desig parent-log-id log-ids)
  (let ((log-id (beliefstate:start-node "PERCEIVE-HUMAN" nil 2 parent-log-id)))
    (beliefstate::add-human-to-node
     desig log-id
     :relative-context-id log-id
     :tf-prefix (conc-strings "human" (write-to-string user-id))
     :srdl-component "http://knowrob.org/kb/openni_human1.owl#iai_human_robot1")
    (ros-info
     :saphari-task-executive
     "start logging ~a with user-id ~a with log-id ~a and parent-log-id ~a"
     desig user-id log-id parent-log-id)
    (set-in-alist user-id log-id log-ids)))

(defun log-acknowledge-lost-human (user-id parent-log-id log-ids)
  (beliefstate:stop-node
   (rest (assoc user-id log-ids))
   :relative-context-id parent-log-id)
  (ros-info
   :saphari-task-executive
   "stop logging person with user-id ~a, log-id ~a, and parent-log-id ~a"
   user-id (rest (assoc user-id log-ids)) parent-log-id)
  (remove-from-alist user-id log-ids))

(defun log-track-new-people (new-people parent-log-id log-ids)
  (if new-people
      (destructuring-bind (new-human &rest other-people) new-people
        (log-track-new-people
         other-people parent-log-id
         (log-track-new-human (car new-human) (rest new-human) parent-log-id log-ids)))
      log-ids))

(defun log-acknowledge-lost-people (lost-people parent-log-id log-ids)
  (if lost-people
      (destructuring-bind (lost-human &rest other-people) lost-people
        (log-acknowledge-lost-people
         other-people parent-log-id
         (log-acknowledge-lost-human (car lost-human) parent-log-id log-ids)))
      log-ids))

(defun log-update-people (current-people last-people parent-log-id log-ids)
  (flet ((excluded-people (subset superset)
           (loop for (id . human) in superset
                 collect (unless (assoc id subset) (cons id human)) into alist
                 finally (return (remove-if-not #'identity alist)))))
    (let ((new-people (excluded-people last-people current-people))
          (lost-people (excluded-people current-people last-people)))
      (log-acknowledge-lost-people
       lost-people parent-log-id (log-track-new-people new-people parent-log-id log-ids)))))

(defun log-people (parent-log-id people-percept)
  (let ((last-people-percept (cpl:make-fluent :value nil))
        (log-ids (cpl:make-fluent :value nil)))
    (unwind-protect
         (cpl:whenever ((cpl:pulsed people-percept))
           (setf (cpl:value log-ids)
                 (log-update-people
                  (cpl:value people-percept)
                  (cpl:value last-people-percept)
                  parent-log-id (cpl:value log-ids)))
           (setf (cpl:value last-people-percept) (cpl:value people-percept)))
      (log-acknowledge-lost-people (cpl:value log-ids) parent-log-id (cpl:value log-ids)))))

(defmacro with-people-monitoring ((parent-log-id human-percept) &body body)
  (alexandria:with-gensyms (human-percept-sym parent-log-id-sym)
    `(let ((,human-percept-sym ,human-percept)
           (,parent-log-id-sym ,parent-log-id))
       (with-people-monitoring-fn
        ,parent-log-id-sym ,human-percept-sym (lambda () ,@body)))))

(defun with-people-monitoring-fn (parent-log-id people-percept main-lambda)
  (with-logging
      ((alexandria:curry #'log-start-people-monitoring parent-log-id)
       (alexandria:rcurry #'log-stop-people-monitoring parent-log-id))
    (cpl:with-tags
      (cpl:pursue
        (log-people log-id people-percept)
        ;; TODO: calc intrusions
        ;; TODO: log intrusions
        ;; TODO: suspend main task
        (:tag main (funcall main-lambda))))))
