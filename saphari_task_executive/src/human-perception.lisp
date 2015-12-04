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

(defun humans-msg->alist (msg)
  (with-fields (humans) msg
    (loop for human across humans
          collect (human-msg->id-and-desig human))))
          
(defun human-msg->id-and-desig (msg)
  (with-fields (userid) msg
    (cons userid (human-designator `((:user-id ,userid))))))

(defun remove-from-alist (key alist &key (test #'equal))
  (remove-if (alexandria:curry test key) alist :key #'car))

(defun set-in-alist (key datum alist)
  (acons key datum (remove-from-alist key alist)))

(defun start-tracking-new-human (user-id desig log-ids)
  (let ((log-id (beliefstate:start-node "PERCEIVE-HUMAN")))
    (beliefstate::add-human-to-node
     desig log-id
     :tf-prefix (conc-strings "human" (write-to-string user-id))
     :srdl-component "http://knowrob.org/kb/openni_human1.owl#iai_human_robot1")
    (ros-info
     :saphari-task-executive
     "start logging ~a with user-id ~a and log-id ~a"
     desig user-id log-id)
    (set-in-alist user-id log-id log-ids)))

(defun stop-tracking-lost-human (user-id log-ids)
  (beliefstate:stop-node (rest (assoc user-id log-ids)))
  (ros-info
   :saphari-task-executive
   "stop logging person with user-id ~a and log-id ~a"
   user-id (rest (assoc user-id log-ids)))
  (remove-from-alist user-id log-ids))

(defun start-tracking-new-humans (new-humans log-ids)
  (if new-humans
      (destructuring-bind (new-human &rest other-humans) new-humans
        (start-tracking-new-humans
         other-humans (start-tracking-new-human (car new-human) (rest new-human) log-ids)))
      log-ids))

(defun stop-tracking-lost-humans (lost-humans log-ids)
  (if lost-humans
      (destructuring-bind (lost-human &rest other-humans) lost-humans
        (stop-tracking-lost-humans
         other-humans (stop-tracking-lost-human (car lost-human) log-ids)))
      log-ids))

(defun excluded-humans (subset superset)
  (loop for (id . human) in superset
        collect (unless (assoc id subset) (cons id human)) into alist
        finally (return (remove-if-not #'identity alist))))

(defun update-humans-tracking (current-humans last-humans log-ids)
  (let ((new-humans (excluded-humans last-humans current-humans))
        (lost-humans (excluded-humans current-humans last-humans)))
    (stop-tracking-lost-humans lost-humans (start-tracking-new-humans new-humans log-ids))))

(defun humans-tracking (humans-percept)
  (let ((last-humans-percept (cpl:make-fluent :value nil))
        (log-ids (cpl:make-fluent :value nil)))
    (unwind-protect
         (cpl:whenever ((cpl:pulsed humans-percept))
           (setf (cpl:value log-ids)
                 (update-humans-tracking
                  (cpl:value humans-percept)
                  (cpl:value last-humans-percept)
                  (cpl:value log-ids)))
           (setf (cpl:value last-humans-percept) (cpl:value humans-percept)))
      (stop-tracking-lost-humans (cpl:value log-ids) (cpl:value log-ids)))))
