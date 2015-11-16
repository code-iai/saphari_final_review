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

(defun instrument-type->mesh-path (type-keyword)
  (case type-keyword
    (:hook "package://saphari_task_executive/models/hospital/surgical-instruments/Hook.dae")
    (:scissor "package://saphari_task_executive/models/hospital/surgical-instruments/Scissors.dae")
    (:rake "package://saphari_task_executive/models/hospital/surgical-instruments/Rake.dae")
    (t nil)))

(defun tool-desig->marker (desig id frame-locked-p)
  (alexandria:when-let* ((type-keyword (desig-prop-value desig :type))
                         (mesh-path (instrument-type->mesh-path type-keyword))
                         (pose-stamped (infer-object-pose desig)))
    (with-fields (header pose) pose-stamped
      (make-message
       "visualization_msgs/Marker"
       :header header
       :ns "cram_instrument_visualization"
       :id id
       :type (symbol-code 'visualization_msgs-msg:Marker :mesh_resource)
       :action (symbol-code 'visualization_msgs-msg:Marker :add)
       :pose pose
       (:x :scale) 1.0 (:y :scale) 1.0 (:z :scale) 1.0
       (:r :color) 0.7 (:g :color) 0.7 (:b :color) 0.7 (:a :color) 1.0
       :frame_locked frame-locked-p
       :mesh_resource mesh-path
       :mesh_use_embedded_materials t))))

(defun delete-all-markers (demo-handle)
  (alexandria:when-let ((pub (getf demo-handle :marker-pub)))
    (publish pub 
             (make-message
              "visualization_msgs/MarkerArray"
              :markers
              (vector (make-message "visualization_msgs/Marker"
                                    :ns "cram_instrument_visualization"
                                    :action 3))))))

(defun publish-tool-markers (demo-handle frame-locked-p &rest desigs)
  (delete-all-markers demo-handle)
  (alexandria:when-let ((markers
                         (remove-if-not #'identity
                                        (loop for desig in desigs
                                              counting t into index
                                              collect (tool-desig->marker desig index frame-locked-p))))
                        (pub (getf demo-handle :marker-pub)))
    (publish pub (make-msg "visualization_msgs/MarkerArray" :markers (coerce markers 'vector)))))
