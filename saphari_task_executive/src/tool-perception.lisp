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
;;; HIGH-LEVEL INTERFACE
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
    (let* ((loc (desig:make-designator
                 'desig:location
                 `((:pose ,(transform-stamped->pose-stamped pose)))))
           (obj (desig:make-designator
                 'desig:object
                 `((:type ,(string->keyword name))
                   (:at ,loc)))))
      obj)))

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
      
;;;
;;; CONVENIENCE CONVERSIONS
;;;

(defun string->keyword (s)
  (declare (type string s))
  (intern (string-upcase s) :keyword))

;; sanity check that we can reach knowrob
;; (prolog-simple "member(A, [1, 2])")

;; query for latest detection of a bowl from Daniel's test episode
;; (prolog-simple "mng_db('test')")
;; (prolog-simple "owl_parse('/tmp/log.owl')")
;; (prolog-simple "knowrob_saphari:saphari_latest_object_detection('Bowl', D)")
