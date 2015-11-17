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

(in-package :saphari-humans-processing)

(defun bodypart-id->radius (id)
  (case (code-symbol 'saphari_msgs-msg:bodypart id)
    ((:lefthand :righthand) 0.13)
    ((:neck :rightchest :leftchest :leftleg
      :rightleg :leftknee :rightknee :leftthigh
      :rightthigh :lefthip :righthip) 0.15)
    ((:head :leftfoot :rightfoot)0.2)
    (:torso 0.3)
    (t 0.1)))

(defun point32-msg->point-msg (msg)
  (declare (type geometry_msgs-msg:point32 msg))
  (with-fields (x y z) msg
    (make-message
     "geometry_msgs/Point"
     :x x :y y :z z)))

(defun radius->scale-msg (radius)
  (make-message
   "geometry_msgs/Vector3"
   :x radius
   :y radius
   :z radius))

(defun default-color-msg ()
  (make-message
   "std_msgs/ColorRGBA"
   :r 0.5 :g 0.5 :b 0.5 :a 1.0))

(defun bodypart-msg->marker-msg (msg user-id &optional (timestamp nil timestamp-supplied-p)
                                           (color (default-color-msg)))
  (declare (type saphari_msgs-msg:bodypart msg))
  (with-fields ((frame-id (frame_id header tf))
                (stamp (stamp header tf))
                id centroid) msg
  (make-message
   "visualization_msgs/Marker"
   (:frame_id :header) frame-id
   (:stamp :header) (if timestamp-supplied-p timestamp stamp)
   (:ns) (concatenate 'string "human" (write-to-string user-id))
   (:id) id
   (:type) (symbol-code 'visualization_msgs-msg:marker :sphere)
   (:action) (symbol-code 'visualization_msgs-msg:marker :add)
   (:position :pose) (point32-msg->point-msg centroid)
   (:w :orientation :pose) 1.0
   :lifetime 5.0
   :scale (radius->scale-msg (bodypart-id->radius id))
   :color color)))
   
(defun human-msg->marker-msg-vector (msg &optional (timestamp nil timestamp-supplied-p))
  (declare (type saphari_msgs-msg:human msg))
  (with-fields ((stamp (stamp header)) userid bodyparts) msg
    (coerce
     (loop for bodypart-msg across bodyparts
           collect (bodypart-msg->marker-msg
                    bodypart-msg userid
                    (if timestamp-supplied-p timestamp stamp)))
     'vector)))

(defun humans-msg->marker-array-msg (msg restamp-p)
  (declare (type saphari_msgs-msg:humans msg))
  (with-fields (humans) msg
    (let* ((stamp (ros-time))
           (list-of-marker-vectors
             (loop for human-msg across humans
                   collect (if restamp-p
                               (human-msg->marker-msg-vector human-msg stamp)
                               (human-msg->marker-msg-vector human-msg))))
           (markers (apply #'concatenate 'vector list-of-marker-vectors)))
      (make-message "visualization_msgs/MarkerArray" :markers markers))))

(defun main-humans-visualization ()
  (with-ros-node ("human_visualization" :spin t)
    (let ((publisher
            (advertise
             "visualization_marker_array"
             "visualization_msgs/MarkerArray"))
          ;; TODO: parameter server lookup
          (restamp-p t))
    (subscribe
     "filtered_humans"
     "saphari_msgs/Humans"
     (lambda (msg)
       (publish
        publisher
        (humans-msg->marker-array-msg msg restamp-p)))))))

           
