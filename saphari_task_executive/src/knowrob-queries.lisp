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

(defun ensure-double-float (num)
  (float num 0.0d0))

(defun knowrob-bindings->pose-msg (trans-bindings rot-bindings)
  (make-msg
   "geometry_msgs/Pose"
   (:x :position) (ensure-double-float (first trans-bindings))
   (:y :position) (ensure-double-float (second trans-bindings))
   (:z :position) (ensure-double-float (third trans-bindings))
   (:x :orientation) (ensure-double-float (first rot-bindings))
   (:y :orientation) (ensure-double-float (second rot-bindings))
   (:z :orientation) (ensure-double-float (third rot-bindings))
   (:w :orientation) (ensure-double-float (fourth rot-bindings))))

(defun query-saphari-next-object ()
  (let* ((query "knowrob_saphari:saphari_next_object(
                    SLOTID, (SLOTTRANS, SLOTROT), OBJCLASS, DESIGID).")
         (before (ros-time))
         (bindings (prolog-simple query))
         (after (ros-time)))
    (ros-info :saphari-next-object "query time: ~a" (- after before))
    (when bindings
      (cut:with-vars-bound (?SLOTID ?SLOTTRANS ?SLOTROT ?OBJCLASS ?DESIGID)
          (cut:lazy-car bindings)
        (list 
         (object-designator
          `((:an :object)
            (:type ,(json-symbol->keyword ?OBJCLASS))
            (:knowrob-id ,(json-symbol->string ?DESIGID))))
         (location-designator
          `((:a :location)
            (:in :sorting-basket)
            (:slot-id ,(json-symbol->string ?SLOTID))
            (:target-object-type ,(json-symbol->keyword ?OBJCLASS))
            (:pose ,(make-msg
                     "geometry_msgs/PoseStamped"
                     (:frame_id :header) "map"
                     (:pose) (knowrob-bindings->pose-msg ?SLOTTRANS ?SLOTROT))))))))))

(defun query-saphari-empty-slots ()
  (let* ((query "knowrob_saphari:saphari_empty_slot((SLOTID, OBJCLASS, (SLOTTRANS, SLOTROT))).")
         (before (ros-time))
         (solutions (cut:force-ll (prolog-simple query))))
    (ros-info :saphari-empty-slot "query time: ~a" (- (ros-time) before))
    (mapcar
     (lambda (solution)
       (cut:with-vars-bound (?SLOTID ?OBJCLASS ?SLOTTRANS ?SLOTROT) solution
         (location-designator `((:a :location)
                                (:in :sorting-basket)
                                (:target-object-type ,(json-symbol->keyword ?OBJCLASS))
                                (:slot-id ,(json-symbol->string ?SLOTID))
                                (:pose ,(make-msg
                                         "geometry_msgs/PoseStamped"
                                         (:frame_id :header) "map"
                                         (:pose) (knowrob-bindings->pose-msg ?SLOTTRANS ?SLOTROT)))))))
     solutions)))

(defun query-saphari-taken-slots ()
  (let* ((query "knowrob_saphari:saphari_taken_slot((SLOTID, OBJCLASS, (SLOTTRANS, SLOTROT))).")
         (before (ros-time))
         (solutions (cut:force-ll (prolog-simple query))))
    (ros-info :saphari-taken-slot "query time: ~a" (- (ros-time) before))
    (mapcar
     (lambda (solution)
       (cut:with-vars-bound (?SLOTID ?OBJCLASS ?SLOTTRANS ?SLOTROT) solution
         (location-designator `((:a :location)
                                (:in :sorting-basket)
                                (:target-object-type ,(json-symbol->keyword ?OBJCLASS))
                                (:slot-id ,(json-symbol->string ?SLOTID))
                                (:pose ,(make-msg
                                         "geometry_msgs/PoseStamped"
                                         (:frame_id :header) "map"
                                         (:pose) (knowrob-bindings->pose-msg ?SLOTTRANS ?SLOTROT)))))))
     solutions)))

(defun infer-target-object-and-location-desigs (perceived-desigs)
  (sleep 0.1) ; TODO: handle the race condition between logging and querying more gracefully
  (alexandria:if-let ((binding (query-saphari-next-object)))
    (destructuring-bind (target-object target-location) binding
      (ros-info :next-object "~a" (desig-prop-value target-object :knowrob-id))
      (dolist (perceived-desig perceived-desigs)
        (ros-info :perceived-desig "~a" (desig-prop-value perceived-desig :knowrob-id)))
      (list
       (find target-object perceived-desigs :test #'desig-descr-included)
       target-location))
    (list nil nil)))

(defun infer-empty-slots (demo-handle)
  (let ((empty-slots (query-saphari-empty-slots))
        (taken-slots (query-saphari-taken-slots)))
    (publish-slot-markers demo-handle empty-slots taken-slots)
    (ros-info :infer-empty-slots "no. empty slots: ~a, no. of taken slots: ~a"
              (length empty-slots) (length taken-slots))
    empty-slots))
