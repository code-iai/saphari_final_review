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

(defun main-humans-filter ()
  (with-ros-node ("humans_filter" :spin t)
    (let* ((publisher (apply #'advertise (ros-configuration :publisher)))
           (callback (curry #'filter-publish-humans-msg publisher))
           (subscription-args (concatenate 'list (ros-configuration :subscriber) (list callback))))
      (apply #'subscribe subscription-args))))
           
(defun ros-configuration (selector)
  (ecase selector
    (:subscriber '("/kinect_traker/user_state" "saphari_msgs/Humans"))
    (:publisher '("filtered_humans" "saphari_msgs/Humans"))))

(defun filter-publish-humans-msg (publisher msg)
  (declare (type saphari_msgs-msg:humans msg))
  (publish publisher (filter-humans-msg msg)))

(defun filter-humans-msg (msg)
  (declare (type saphari_msgs-msg:humans msg))
  (with-fields (humans observed_user_ids) msg
    (let ((filtered-humans
            (loop for human across humans collect (filter-human-msg human))))
      (make-message
       "saphari_msgs/Humans"
       :humans (coerce filtered-humans 'vector)
       :observed_user_ids observed_user_ids))))

(defun filter-human-msg (msg)
  (declare (type saphari_msgs-msg:human msg))
  (with-fields (bodyparts header userid) msg
    (make-message
     "saphari_msgs/Human"
     :header header
     :userid userid
     :bodyparts (remove-if #'empty-bodypart-msg-p bodyparts))))

(defun empty-bodypart-msg-p (msg)
  (declare (type saphari_msgs-msg:bodypart msg))
  (with-fields ((child-frame-id (child_frame_id tf))) msg
    (string= child-frame-id "")))
