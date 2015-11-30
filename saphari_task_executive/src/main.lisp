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

(defun make-demo-handle (&optional (sim-p t))
  (list
   :json-prolog nil
   :sim-p sim-p
   :tool-perception (ros-interface-tool-perception)
   :beasty (make-and-init-beasty-handle sim-p)
   :wsg50 (make-wsg-handle)
   :tf-broadcaster (advertise "/tf" "tf2_msgs/TFMessage")
   :tf-listener (make-instance 'cl-tf2:buffer-client)
   :marker-pub (advertise "visualization_marker_array" "visualization_msgs/MarkerArray")
   ))

(defun main ()
  (with-ros-node ("cram")
    (let ((demo-handle (make-demo-handle)))
      (beliefstate::init-semrec)
      (beliefstate:enable-logging t)
      (cpl:top-level
        ;; TODO: human reactivity
        ;; TODO: failure handling
        (let* ((empty-slots-fluent (cpl:make-fluent :value (infer-empty-slots demo-handle))))
          (cpl:pursue
            (cpl:wait-for (cpl:not empty-slots-fluent))
            (loop-at-most-every 1
              (setf (cpl:value empty-slots-fluent) (infer-empty-slots demo-handle))
              ;; TODO: change into nice perception plan
              (lookat-pickup-zone demo-handle)
              (alexandria:when-let* ((object-desigs (trigger-tool-perception demo-handle)))
                (multiple-value-bind (target-object target-location)
                    (infer-target-object-and-location-desigs object-desigs)
                  (let ((updated-target-object (grasp-object demo-handle target-object)))
                    (put-down demo-handle updated-target-object target-location))))))))
      (beliefstate:extract-files))))

;;;
;;; TEMPORARY DEBUG/DEVEL CODE
;;;

(defun make-target-object (type-keyword slot-id pose)
  (object-designator
   `((:an :object)
     (:type ,type-keyword)
     (:at ,(location-designator
            `((:a :location)
              (:slot-id ,slot-id)
              (:pose ,(make-msg
                       "geometry_msgs/PoseStamped"
                       (:frame_id :header) "sorting_basket"
                       :pose (pose->pose-msg pose)))))))))

(defun visualize-goal-objects (goal-id demo-handle)
  (apply #'publish-tool-markers demo-handle nil (target-objects goal-id)))

(defun target-objects (goal-id)
  (case goal-id
    (1
     (list
      (make-target-object :bandage-scissors 0 (make-pose (make-3d-vector 0.28 0.07 0.02) (make-identity-rotation)))
      (make-target-object :scalpel 1 (make-pose (make-3d-vector 0.34 -0.05 0.02) (make-quaternion 0 0 -0.707107 0.707107)))
      (make-target-object :scalpel 2 (make-pose (make-3d-vector 0.29 -0.05 0.02) (make-quaternion 0 0 -0.707107 0.707107)))
      (make-target-object :small-clamp  3 (make-pose (make-3d-vector 0.05 0.04 0.02) (make-quaternion 0 0 -0.707107 0.707107)))
      (make-target-object :retractor 4 (make-pose (make-3d-vector 0.14 -0.08 0.02) (make-identity-rotation)))
      (make-target-object :big-clamp 5 (make-pose (make-3d-vector 0.16 -0.02 0.02) (make-identity-rotation)))
      (make-target-object :pincers 6 (make-pose (make-3d-vector 0.13 0.03 0.02) (make-identity-rotation)))
      (make-target-object :pincers 7 (make-pose (make-3d-vector 0.13 0.08 0.02) (make-identity-rotation)))))
    (t nil)))

(defun desig-pose-in-map (demo-handle desig)
  (tf2-transform-pose-stamped-msg
   (getf demo-handle :tf-listener)
   (infer-object-pose desig)
   "map"))

(defun random-password (length)
  (let ((chars "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"))
    (coerce (loop repeat length collect (aref chars (random (length chars))))
            'string)))

(defun object-desig->slot-owl (demo-handle basket-hash desig)
  (alexandria:when-let ((type-keyword (desig-prop-value desig :type))
                        (pose (desig-pose-in-map demo-handle desig))
                        (slot-hash (random-password 8))
                        (transform-hash (random-password 8))
                        (template-string
                         "
<!-- http://knowrob.org/kb/saphari.owl#BasketSlot_~a -->
<owl:NamedIndividual rdf:about=\"&saphari;BasketSlot_~a\">
  <rdf:type rdf:resource=\"&saphari;BasketSlot\"/>
  <knowrob:perceptionResponse>~a</knowrob:perceptionResponse>
  <knowrob:physicalPartOf rdf:resource=\"&saphari;Basket_~a\"/>
</owl:NamedIndividual>

<owl:NamedIndividual rdf:about=\"&saphari;Transformation_~a\">
  <rdf:type rdf:resource=\"&knowrob;Transformation\"/>
  <knowrob:quaternion rdf:datatype=\"&xsd;string\">~f ~f ~f ~f</knowrob:quaternion>
  <knowrob:translation rdf:datatype=\"&xsd;string\">~f ~f ~f</knowrob:translation>
</owl:NamedIndividual>

<owl:NamedIndividual rdf:about=\"&knowrob;SemanticMapPerception_~a\">
  <rdf:type rdf:resource=\"&knowrob;SemanticMapPerception\"/>
  <knowrob:eventOccursAt rdf:resource=\"&saphari;Transformation_~a\"/>
  <knowrob:startTime rdf:resource=\"&saphari;timepoint_0000000001\"/>
  <knowrob:objectActedOn rdf:resource=\"&saphari;BasketSlot_~a\"/>
</owl:NamedIndividual>
"))
    (with-slots (orientation origin) pose
      (with-slots ((qx x) (qy y) (qz z) (qw w)) orientation
        (with-slots (x y z) origin
          (format
           nil template-string
           slot-hash slot-hash (symbol-name type-keyword) basket-hash transform-hash
           qx qy qz qw
           x y z
           transform-hash transform-hash slot-hash))))))

(defun object-desigs->basket-owl (demo-handle desigs &key (princ-result t) (return-result nil))
  (let* ((basket-hash (random-password 8))
         (template-string
          "<!-- http://knowrob.org/kb/saphari.owl#Basket_~a -->
<owl:NamedIndividual rdf:about=\"&saphari;Basket_~a\">
  <rdf:type rdf:resource=\"&saphari;Basket\"/>
</owl:NamedIndividual>")
         (result
           (apply
            #'concatenate
            'string
            (format nil template-string basket-hash basket-hash)
            (mapcar (alexandria:curry #'object-desig->slot-owl demo-handle basket-hash) desigs))))
    (when princ-result (princ result))
    (when return-result result)))


(defparameter *dh* nil)

(defun bringup-scripting-environment ()
  (start-ros-node "cram")
  (setf *dh* (make-demo-handle)))

(defun fill-knowrob-with-percepts (demo-handle)
  (cpl:top-level
    (lookat-pickup-zone demo-handle)
    (trigger-tool-perception demo-handle)))


