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

;;;
;;; LOG HUMANS PERCEPTIONS
;;;

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

(defun log-people (parent-log-id people-percept-fluent)
  (let ((last-people-percept (cpl:make-fluent :value nil))
        (log-ids (cpl:make-fluent :value nil)))
    (unwind-protect
         (cpl:whenever ((cpl:pulsed people-percept-fluent))
           (setf (cpl:value log-ids)
                 (log-update-people
                  (cpl:value people-percept-fluent)
                  (cpl:value last-people-percept)
                  parent-log-id (cpl:value log-ids)))
           (setf (cpl:value last-people-percept) (cpl:value people-percept-fluent)))
      (log-acknowledge-lost-people (cpl:value log-ids) parent-log-id (cpl:value log-ids)))))

;;;
;;; LOG INTRUSIONS
;;;

(defun sample-intrusions ()
  (let ((human2 (make-human 2))
        (human3 (make-human 3)))
    (list
     (list
      (make-intrusion human2 "head")
      (make-intrusion human2 "neck")
      (make-intrusion human3 "head"))
     (list
      (make-intrusion human2 "head")
      (make-intrusion human3 "neck")))))

(defun sample-log-ids ()
  (list
   (list
    (cons (bodypart-frame-id 2 "head") 17)
    (cons (bodypart-frame-id 2 "neck") 18)
    (cons (bodypart-frame-id 3 "head") 5))
   (list
    (cons (bodypart-frame-id 2 "head") 17)
    (cons (bodypart-frame-id 3 "neck") 42))))

(defun log-new-intrusion (intrusion parent-log-id log-ids)
  (let* ((intrusion-id (car intrusion))
         (bodypart (cddr intrusion))
         (human (cadr intrusion))
         (desig (rest human)))
    (let ((log-id (beliefstate:start-node "HUMAN-INTRUSION" (list :_bodypart bodypart) 2 parent-log-id)))
      (beliefstate::add-human-to-node
       desig log-id
       :relative-context-id log-id
       :tf-prefix (conc-strings "human" (write-to-string (car human)))
       :srdl-component "http://knowrob.org/kb/openni_human1.owl#iai_human_robot1")
      (ros-info :start-intrusion "~a with log-id ~a" intrusion-id log-id)
      (set-in-alist intrusion-id log-id log-ids))))

(defun log-finished-intrusion (intrusion parent-log-id log-ids)
  (let ((intrusion-id (car intrusion)))
    (beliefstate:stop-node
     (rest (assoc intrusion-id log-ids :test #'string=))
     :relative-context-id parent-log-id)
    (ros-info
     :finish-intrusion "~a, parent-log-id ~a" intrusion-id parent-log-id)
    (remove-from-alist intrusion-id log-ids)))

(defun log-new-intrusions (intrusions parent-log-id log-ids)
  (if intrusions
      (destructuring-bind (intrusion &rest other-intrusions) intrusions
        (log-new-intrusions 
         other-intrusions parent-log-id
         (log-new-intrusion intrusion parent-log-id log-ids)))
      log-ids))

(defun log-finished-intrusions (intrusions parent-log-id log-ids)
  (if intrusions
      (destructuring-bind (intrusion &rest other-intrusions) intrusions
        (log-finished-intrusions
         other-intrusions parent-log-id
         (log-finished-intrusion intrusion parent-log-id log-ids)))
      log-ids))

(defun log-update-intrusions (current-intrusions last-intrusions parent-log-id log-ids)
  (flet ((excluded-intrusions (subset superset)
           (loop for intrusion in superset
                 collect (unless (assoc (car intrusion) subset :test #'string=) intrusion) into alist
                 finally (return (remove-if-not #'identity alist)))))
    (let ((new-intrusions (excluded-intrusions last-intrusions current-intrusions))
          (finished-intrusions (excluded-intrusions current-intrusions last-intrusions)))
      (log-finished-intrusions
       finished-intrusions parent-log-id (log-new-intrusions new-intrusions parent-log-id log-ids)))))

(defun log-intrusions (parent-log-id intrusions-fluent)
  (let ((last-intrusions (cpl:make-fluent :value nil))
        (log-ids (cpl:make-fluent :value nil)))
    (unwind-protect
         (cpl:whenever ((cpl:pulsed intrusions-fluent))
           (setf (cpl:value log-ids)
                 (log-update-intrusions
                  (cpl:value intrusions-fluent)
                  (cpl:value last-intrusions)
                  parent-log-id (cpl:value log-ids)))
           (setf (cpl:value last-intrusions) (cpl:value intrusions-fluent)))
      (log-finished-intrusions (cpl:value log-ids) parent-log-id (cpl:value log-ids)))))

;;;
;;; CALC INTRUSIONS
;;;

(defun bodypart-frame-id (user-id bodypart-name)
  (declare (type number user-id)
           (type string bodypart-name))
  (conc-strings "human" (write-to-string user-id) "/" bodypart-name))

(defun make-intrusion (human bodypart-name)
  (cons
   (bodypart-frame-id (car human) bodypart-name)
   (cons human bodypart-name)))

(defun intruding-bodypart-p (demo-handle user-id bodypart-name threshold)
  (declare (type list demo-handle)
           (type string bodypart-name)
           (type number user-id threshold))
  (let* ((transform-stamped
           (tf2-lookup
            (getf demo-handle :tf-listener)
            "gripper_tool_frame"
            (bodypart-frame-id user-id bodypart-name)))
         (translation (translation (cl-tf2::transform transform-stamped))))
    (< (abs (v-norm translation)) (abs threshold))))

(defun bodypart-thresholds ()
  (list
   (cons "head" 0.4)
   (cons "left_hand" 0.3)
   (cons "right_hand" 0.3)))

(defun intruding-bodyparts-for-human (demo-handle user-id
                                      &optional (bodypart-thresholds (bodypart-thresholds)))
  (declare (type list demo-handle bodypart-thresholds)
           (type number user-id))
  (remove-if-not
   #'identity
   (mapcar
    (lambda (bodypart-threshold)
      (let ((bodypart-name (car bodypart-threshold))
            (threshold (rest bodypart-threshold)))
        (when (intruding-bodypart-p demo-handle user-id bodypart-name threshold)
          bodypart-name)))
    bodypart-thresholds)))

(defun intrusions-for-human (demo-handle human &optional bodypart-thresholds)
  (mapcar
   (alexandria:curry #'make-intrusion human)
   (if bodypart-thresholds
       (intruding-bodyparts-for-human demo-handle (car human) bodypart-thresholds)
       (intruding-bodyparts-for-human demo-handle (car human)))))

(defun intrusions-for-people (demo-handle people &optional bodypart-thresholds)
  (apply
   #'conc-lists
   (remove-if-not
    #'identity
    (mapcar
     (lambda (human)
       (intrusions-for-human demo-handle human bodypart-thresholds))
     people))))

;;;
;;; ACTUAL MONITORING MACRO
;;;

(defmacro with-people-monitoring ((parent-log-id demo-handle &optional bodypart-thresholds) &body body)
  (alexandria:with-gensyms (parent-log-id-sym demo-handle-sym bodypart-thresholds-sym)
    `(let ((,demo-handle-sym ,demo-handle)
           (,parent-log-id-sym ,parent-log-id)
           (,bodypart-thresholds-sym ,bodypart-thresholds))
       (with-people-monitoring-fn
        ,parent-log-id-sym ,demo-handle-sym (lambda () ,@body) ,bodypart-thresholds-sym))))

(defun with-people-monitoring-fn (parent-log-id demo-handle main-lambda &optional bodypart-thresholds)
  (with-logging
      ((alexandria:curry #'log-start-people-monitoring parent-log-id)
       (alexandria:rcurry #'log-stop-people-monitoring parent-log-id))
    (let* ((people-percept-fluent (getf demo-handle :humans-percept-fluent))
           (intrusions-fluent (cpl:make-fluent :value nil))
           ;; (intrusions-fluent
           ;;   (cpl:fl-funcall #'intrusions-for-people demo-handle people-percept-fluent bodypart-thresholds))
           )
      (cpl:with-tags
        (cpl:pursue
          (log-people log-id people-percept-fluent)

          (log-intrusions log-id intrusions-fluent)
          
          ;; TODO: to be replaced by something cooler
          (loop
            (setf (cpl:value intrusions-fluent)
                  (intrusions-for-people demo-handle (cpl:value people-percept-fluent) bodypart-thresholds))
            (cpl:sleep 0.3))

          ;; TODO: suspend task
          ;; (cpl:whenever ((cpl:pulsed intrusions-fluent))
          ;;   ;; TODO: log task suspension?
          ;;   ;; TODO: suspension
          ;;   (when (cpl:value intrusions-fluent)
          ;;     (ros-info :current-intrusions "~a" (cpl:value intrusions-fluent))))

          ;; TODO: make gensym for main-symbol
          (:tag main (funcall main-lambda)))))))
