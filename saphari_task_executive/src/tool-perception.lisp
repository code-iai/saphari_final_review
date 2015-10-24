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
;;; KNOWROB UTILS
;;;

(defun json-symbol->keyword (json-symbol)
  (declare (type symbol json-symbol))
  (string->keyword (json-symbol->string json-symbol)))

(defun json-symbol->string (json-symbol)
  (declare (type symbol json-symbol))
  (remove #\' (symbol-name json-symbol)))

;;;
;;; QUERY KNOWROB
;;;

(defun keywords->knowrob-string-list (&rest keywords)
  (let* ((strings (mapcar #'symbol-name keywords))
         (quoted-strings (mapcar (lambda (s) (conc-strings "'" s "'")) strings))
         (comma-strings
           (apply #'conc-strings
                  (mapcar (lambda (s) (conc-strings s ",")) quoted-strings))))
    (conc-strings "[" (remove #\, comma-strings :from-end t :count 1) "]")))

(defun query-latest-instrument-detections (&rest class-keywords)
  (let ((query-string
          (conc-strings "knowrob_saphari:saphari_latest_object_detections("
                        (apply #'keywords->knowrob-string-list class-keywords)
                        ",_Detections),"
                        "member(_Detection, _Detections),"
                        "mng_designator(_Detection, _ObjJava),"
                        "mng_designator_props(_Detection, _ObjJava, ['TYPE'], DESIGTYPE),"
                        "mng_designator_props(_Detection, _ObjJava, ['AT', 'POSE'], _DesigPose),"
                        "jpl_get(_DesigPose, 'frameID', FRAMEID),"
                        "jpl_get(_DesigPose, 'timeStamp', _TimeStampIso),"
                        "jpl_call(_TimeStampIso, 'toSeconds', [], TIMESTAMP),"
;;                        "jpl_call(_TimeStampIso, 'totalNsecs', [], TIMESTAMP),"
                        "jpl_call(_DesigPose, 'getData', [], _Pose),"
                        "knowrob_coordinates:matrix4d_to_list(_Pose, _PoseList),"
                        "matrix_rotation(_PoseList, [QW, QX, QY, QZ]),"
                        "matrix_translation(_PoseList, [X, Y, Z]).")))
    (let ((bindings (cut:force-ll (prolog-simple query-string))))
      (mapcar (lambda (binding)
                (cut:with-vars-bound (?QW ?QX ?QY ?QZ
                                          ?X ?Y ?Z
                                          ?TIMESTAMP ?FRAMEID
                                          ?DESIGTYPE) binding
                  (type-and-pose-stamped->obj-desig
                   (json-symbol->keyword ?DESIGTYPE)
                   (make-msg
                    "geometry_msgs/PoseStamped"
                    (:stamp :header) ?TIMESTAMP
                    (:frame_id :header) (json-symbol->string ?FRAMEID)
                    (:x :position :pose) ?X
                    (:y :position :pose) ?Y
                    (:z :position :pose) ?Z
                    (:x :orientation :pose) ?QX
                    (:y :orientation :pose) ?QY
                    (:z :orientation :pose) ?QZ
                    (:w :orientation :pose) ?QW))))
              bindings))))

;;;
;;; HIGH-LEVEL PLAN INTERFACE
;;;

(cpl-impl:def-cram-function perceive-surgical-instruments ()
  (cpl-desig-supp:with-designators ((obj-desig (desig:object '((:type :surgical-instrument)))))
    (let* ((logging-id (on-prepare-perception-request obj-desig))
           (desigs (tool-perception-response->object-desigs
                    (trigger-tool-perception))))
      (on-finish-perception-request logging-id desigs)
      desigs)))

;;;
;;; LOGGING INTERFACE
;;;

(defun on-prepare-perception-request (request)
  (declare (type desig:object-designator request))
  (let ((id (beliefstate:start-node
             "UIMA-PERCEIVE"
             (cram-designators:description request))))
    (beliefstate:add-designator-to-node request id :annotation "perception-request")
    id))

(defun on-finish-perception-request (id results)
  (dolist (desig results)
    (beliefstate:add-designator-to-node
     desig id :annotation "perception-result"))
;  (beliefstate:add-topic-image-to-active-node cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node id :success (not (eql results nil))))

;;;
;;; ROS INTERFACING
;;;

(defun trigger-tool-perception ()
  (call-service
   "/tool_detector/detect_tools"
   "saphari_tool_detector/DetectTools"))

;;;
;;; ROS MESSAGE AND DESIGNATOR CONVERSIONS
;;;

(defun tool-perception-response->object-desigs (tool-perception-response)
  (declare (type saphari_tool_detector-srv:detecttools-response tool-perception-response))
  (with-fields (tools) tool-perception-response
    (mapcar #'tool-percept->object-desig (coerce tools 'list))))
             
(defun tool-percept->object-desig (tool-percept)
  (declare (type saphari_tool_detector-msg:tool tool-percept))
  (with-fields (name pose) tool-percept
    (type-and-pose-stamped->obj-desig
     (string->keyword name)
     (transform-stamped->pose-stamped pose))))

(defun type-and-pose-stamped->obj-desig (object-type pose-stamped)
  (declare (type keyword object-type)
           (type geometry_msgs-msg:posestamped pose-stamped))
  (desig:make-designator
   'desig:object
   `((:type ,object-type)
     (:at ,(pose-stamped->loc-desig pose-stamped)))))

(defun pose-stamped->loc-desig (pose-stamped)
  (declare (type geometry_msgs-msg:posestamped pose-stamped))
  (desig:make-designator 'desig:location `((:pose ,pose-stamped))))

;;;
;;; ROS MESSAGE CONVERSIONS
;;;

(defun transform->pose (transform)
  "Converts `transform' of type geometry_msgs/Transform into an
 instance of type geometry_msgs/Pose without changing `transform'."
  (declare (type geometry_msgs-msg:transform transform))
  (with-fields (translation rotation) transform
    (make-msg
     "geometry_msgs/Pose"
     :position translation
     :orientation rotation)))
                  
(defun transform-stamped->pose-stamped (transform-stamped)
  "Converts `transform-staped' of type geometry_msgs/TransformStamped
 into an instance of type geometry_msgs/PoseStamped without changing
`transform-stamped'."
  (declare (type geometry_msgs-msg:transformstamped transform-stamped))
  (with-fields (header transform) transform-stamped
    (make-msg "geometry_msgs/PoseStamped"
              :header header
              :pose (transform->pose transform))))
