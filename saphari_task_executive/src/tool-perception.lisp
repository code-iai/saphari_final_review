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
;;; NUMBER UTILS
;;;

(defun metric-input->cm-keyword (metric-input)
  "Returns the keyword representing 'metric-input'. Assumes
 that 'metric-input' is either in centimeters and INTEGER, or in
 meters and RATIO or FLOAT, or meters of centimeters
 and STRING. NOTE: Returns 'metric-input' if it is a keyword.

 EXAMPLES:
 STE> (metric-input->cm-keyword 0.2)
 :20cm
 STE> (metric-input->cm-keyword 20)
 :20cm
 STE> (metric-input->cm-keyword 2/10)
 :20cm
 STE> (metric-input->cm-keyword \"20\")
 :20cm
 STE> (metric-input->cm-keyword \"0.2\")
 :20cm
"
  (if (keywordp metric-input)
      metric-input
      (metric-input->cm-keyword
       (etypecase metric-input
         (string (read-from-string metric-input))
         (ratio (float metric-input))
         (float (round (* 100 metric-input)))
         (integer (string->keyword (conc-strings (write-to-string metric-input) "cm")))))))

;;;
;;; MAGIC NUMBERS
;;;

(defun 20cm-lookat-pickup-config ()
  (list -0.162 0.870 -0.185 -1.181 0.165 1.088 0.432))

(defun 30cm-lookat-pickup-config ()
  (list -0.159 0.762 -0.200 -1.092 0.149 1.284 0.444))

(defun 40cm-lookat-pickup-config ()
  (list -0.166 0.708 -0.209 -0.919 0.141 1.509 0.457))

(defun 50cm-lookat-pickup-config ()
  (list -0.195 0.733 -0.207 -0.611 0.148 1.79 0.474))

(defun lookat-pickup-config (metric-input)
  (case (metric-input->cm-keyword metric-input)
    (:20cm (20cm-lookat-pickup-config))
    (:30cm (30cm-lookat-pickup-config))
    (:40cm (40cm-lookat-pickup-config))
    (:50cm (50cm-lookat-pickup-config))))

(defun lookat-sorting-basket-config ()
  (list
   0.5003623962402344
   0.8569384217262268
   0.08925693482160568
   -1.1276057958602905
   -0.0697876513004303
   1.164320468902588
   0.5868684649467468))

;;;
;;; QUERY KNOWROB
;;;

(defun query-knowrob-tool-perception (&rest class-keywords)
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
                  (properties->obj-desig
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
                    (:w :orientation :pose) ?QW)
                   ;; TODO: get confidence from KNOWROB
                   0.2
                   ;; TODO: get that extra description from knowrob
                   '((:on :table)))))
              bindings))))

;;;
;;; HIGH-LEVEL PLAN INTERFACE
;;;

(cpl-impl:def-cram-function trigger-tool-perception (demo-handle)
  (cpl-desig-supp:with-designators ((obj-desig (desig:object '((:type :surgical-instrument)))))
    (let* ((logging-id (on-prepare-perception-request obj-desig))
           (desigs (tool-perception-response->object-desigs
                    (apply #'call-service (getf demo-handle :tool-perception))
                    '((:on :table)))))
      (on-finish-perception-request logging-id desigs)
      (apply #'publish-tool-markers demo-handle nil desigs)
      (publish-tool-poses-to-tf demo-handle desigs)
      desigs)))

(cpl-impl:def-cram-function query-tool-perception (demo-handle &rest desigs)
  (if (getf demo-handle :json-prolog)
      ;; TODO: desig->class-keywords
      ;; TODO: equate?
      (apply #'query-knowrob-tool-perception desigs)
      (cpl:fail "Tool query failed: json-prolog disabled.")))

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
;;; ROS INTERFACE
;;;

(defun ros-interface-tool-perception ()
  (list
   "/tool_detector/detect_tools"
   "saphari_tool_detector/DetectTools"))
