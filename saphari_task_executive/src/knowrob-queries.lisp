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

;; (defun query-knowrob-tool-perception (&rest class-keywords)
;;   (let ((query-string
;;           (conc-strings "knowrob_saphari:saphari_latest_object_detections("
;;                         (apply #'keywords->knowrob-string-list class-keywords)
;;                         ",_Detections),"
;;                         "member(_Detection, _Detections),"
;;                         "mng_designator(_Detection, _ObjJava),"
;;                         "mng_designator_props(_Detection, _ObjJava, ['TYPE'], DESIGTYPE),"
;;                         "mng_designator_props(_Detection, _ObjJava, ['AT', 'POSE'], _DesigPose),"
;;                         "jpl_get(_DesigPose, 'frameID', FRAMEID),"
;;                         "jpl_get(_DesigPose, 'timeStamp', _TimeStampIso),"
;;                         "jpl_call(_TimeStampIso, 'toSeconds', [], TIMESTAMP),"
;; ;;                        "jpl_call(_TimeStampIso, 'totalNsecs', [], TIMESTAMP),"
;;                         "jpl_call(_DesigPose, 'getData', [], _Pose),"
;;                         "knowrob_coordinates:matrix4d_to_list(_Pose, _PoseList),"
;;                         "matrix_rotation(_PoseList, [QW, QX, QY, QZ]),"
;;                         "matrix_translation(_PoseList, [X, Y, Z]).")))
;;     (let ((bindings (cut:force-ll (prolog-simple query-string))))
;;       (mapcar (lambda (binding)
;;                 (cut:with-vars-bound (?QW ?QX ?QY ?QZ
;;                                           ?X ?Y ?Z
;;                                           ?TIMESTAMP ?FRAMEID
;;                                           ?DESIGTYPE) binding
;;                   (properties->obj-desig
;;                    (json-symbol->keyword ?DESIGTYPE)
;;                    (make-msg
;;                     "geometry_msgs/PoseStamped"
;;                     (:stamp :header) ?TIMESTAMP
;;                     (:frame_id :header) (json-symbol->string ?FRAMEID)
;;                     (:x :position :pose) ?X
;;                     (:y :position :pose) ?Y
;;                     (:z :position :pose) ?Z
;;                     (:x :orientation :pose) ?QX
;;                     (:y :orientation :pose) ?QY
;;                     (:z :orientation :pose) ?QZ
;;                     (:w :orientation :pose) ?QW)
;;                    ;; TODO: get confidence from KNOWROB
;;                    0.2
;;                    ;; TODO: get that extra description from knowrob
;;                    '((:on :table)))))
;;               bindings))))

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
  (let ((query "knowrob_saphari:saphari_next_object(
                    SLOTID, (SLOTTRANS, SLOTROT), OBJCLASS, DESIGID)"))
    (cut:with-vars-bound (?SLOTID ?SLOTTRANS ?SLOTROT ?OBJCLASS ?DESIGID)
        (cut:lazy-car (prolog-simple query))
      (values 
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
                   (:pose) (knowrob-bindings->pose-msg ?SLOTTRANS ?SLOTROT)))))))))

(defun infer-target-object-and-location-desigs (perceived-desigs)
  (multiple-value-bind (target-object target-location) (query-saphari-next-object)
    (values
     (find target-object perceived-desigs :test #'desig-descr-included)
     target-location)))
