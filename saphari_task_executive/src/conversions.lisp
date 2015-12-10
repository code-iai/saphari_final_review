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
;;; ROS MESSAGE AND DESIGNATOR CONVERSIONS
;;;

(defun tool-perception-response->object-desigs (tool-perception-response extra-descr)
  (declare (type saphari_tool_detector-srv:detecttools-response tool-perception-response))
  (with-fields (tools) tool-perception-response
    (mapcar (alexandria:rcurry #'tool-percept->object-desig extra-descr) (coerce tools 'list))))
             
(defun tool-percept->object-desig (tool-percept extra-descr)
  (declare (type saphari_tool_detector-msg:tool tool-percept))
  (with-fields (name pose confidence) tool-percept
    (properties->obj-desig
     (string->keyword name)
     (transform-stamped-msg->pose-stamped-msg pose)
     confidence
     extra-descr)))

(defun properties->obj-desig (object-type pose-stamped confidence extra-descr)
  (declare (type keyword object-type)
           (type geometry_msgs-msg:posestamped pose-stamped))
  (object-designator
   `((:an :object)
     (:type ,object-type)
     (:confidence ,confidence)
     (:at ,(pose-stamped->loc-desig pose-stamped extra-descr)))))

(defun pose-stamped->loc-desig (pose-stamped extra-descr)
  (declare (type geometry_msgs-msg:posestamped pose-stamped))
  (location-designator (conc-lists `((:a :location)(:pose ,pose-stamped)) extra-descr)))

(defun make-human (user-id &optional knowrob-id)
  (let ((descr (if knowrob-id
                   `((:user-id ,user-id)(:knowrob-id ,knowrob-id))
                   `((:user-id ,user-id)))))
    (cons user-id (human-designator descr))))

(defun humans-msg->alist (msg)
  (declare (type saphari_msgs-msg:humans msg))
  (with-fields (observed_user_ids) msg
    (loop for user-id across observed_user_ids
          collect (make-human user-id))))

;;;
;;; ROS MESSAGE CONVERSIONS
;;;

(defun vector3->point (vector3)
  (declare (type geometry_msgs-msg:vector3 vector3))
  (with-fields (x y z) vector3
    (make-msg
     "geometry_msgs/Point"
     :x x :y y :z z)))

(defun point->vector3 (point)
  (declare (type geometry_msgs-msg:point point))
  (with-fields (x y z) point
    (make-msg
     "geometry_msgs/Vector3"
     :x x :y y :z z)))

(defun transform-msg->pose-msg (transform)
  "Converts `transform' of type geometry_msgs/Transform into an
 instance of type geometry_msgs/Pose without changing `transform'."
  (declare (type geometry_msgs-msg:transform transform))
  (with-fields (translation rotation) transform
    (make-msg
     "geometry_msgs/Pose"
     :position (vector3->point translation)
     :orientation rotation)))

(defun pose-msg->transform-msg (pose)
  "Converts `pose' of type geometry_msgs/Pose into an instance
 of type geometry_msgs/Transform without changing `pose'."
  (declare (type geometry_msgs-msg:pose pose))
  (with-fields (position orientation) pose
    (make-message
     "geometry_msgs/Transform"
     :translation (point->vector3 position)
     :rotation orientation)))
                  
(defun transform-stamped-msg->pose-stamped-msg (transform-stamped)
  "Converts `transform-staped' of type geometry_msgs/TransformStamped
 into an instance of type geometry_msgs/PoseStamped without changing
`transform-stamped'."
  (declare (type geometry_msgs-msg:transformstamped transform-stamped))
  (with-fields (header transform) transform-stamped
    (make-msg "geometry_msgs/PoseStamped"
              :header header
              :pose (transform-msg->pose-msg transform))))

(defun pose-stamped-msg->transform-stamped-msg (pose-stamped child-frame-id)
  "Converts `pose-staped' of type geometry_msgs/PoseStamped into
 an instance of type geometry_msgs/TransformStamped using `child-frame-id'.
 Note: Both inputs will remain unchanged."
  (with-fields (header pose) pose-stamped
    (make-msg "geometry_msgs/TransformStamped"
              :header header
              :child_frame_id child-frame-id
              :transform (pose-msg->transform-msg pose))))

;;;
;;; ROS MESSAGE AND CL-TRANSFORMS CONVERSIONS
;;;

(defun wrench->wrench-msg (wrench)
  (declare (type cl-transforms:wrench wrench))
  (with-slots (translation rotation) wrench
    (make-msg
     "geometry_msgs/Wrench"
     :force (3d-vector->vector3-msg translation)
     :torque (3d-vector->vector3-msg rotation))))

(defun point-msg->3d-vector (msg)
  (declare (type geometry_msgs-msg:point msg))
  (with-fields (x y z) msg
    (cl-transforms:make-3d-vector x y z)))

(defun 3d-vector->point-msg (3d-vector)
  (declare (type cl-transforms:3d-vector 3d-vector))
  (with-slots (x y z) 3d-vector
    (make-msg "geometry_msgs/Point" :x x :y y :z z)))

(defun 3d-vector->vector3-msg (3d-vector)
  (declare (type cl-transforms:3d-vector 3d-vector))
  (with-slots (x y z) 3d-vector
    (make-msg "geometry_msgs/Vector3" :x x :y y :z z)))

(defun quaternion-msg->quaternion (msg)
  (declare (type geometry_msgs-msg:quaternion msg))
  (with-fields (x y z w) msg
    (cl-transforms:make-quaternion x y z w)))

(defun quaternion->quaterion-msg (quaternion)
  (declare (type cl-transforms:quaternion))
  (with-slots (x y z w) quaternion
    (make-message
     "geometry_msgs/Quaternion"
     :x x :y y :z z :w w)))

(defun pose-msg->transform (msg)
  (declare (type geometry_msgs-msg:pose msg))
  (with-fields (position orientation) msg
    (cl-transforms:make-transform
     (point-msg->3d-vector position)
     (quaternion-msg->quaternion orientation))))

(defun pose-msg->pose (msg)
  (cl-transforms:transform->pose (pose-msg->transform msg)))

(defun transform->pose-msg (transform)
  (declare (type cl-transforms:transform))
  (with-slots (translation rotation) transform
    (make-message
     "geometry_msgs/Pose"
     :position (3d-vector->point-msg translation)
     :orientation (quaternion->quaterion-msg rotation))))

(defun pose->pose-msg (pose)
  (transform->pose-msg (cl-transforms:pose->transform pose)))

(defun pose-stamped-msg->transform (msg)
  (declare (type geometry_msgs-msg:posestamped msg))
  (with-fields (pose) msg
    (pose-msg->transform pose)))

(defun transform->pose-stamped-msg (transform frame-id)
  (declare (type cl-transforms:transform transform)
           (type string frame-id))
  (make-message
   "geometry_msgs/PoseStamped"
   (:frame_id :header) frame-id
   (:stamp :header) (ros-time)
   :pose (transform->pose-msg transform)))
