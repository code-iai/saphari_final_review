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

(defun tf2-lookup (tf frame-id child-frame-id)
  (handler-case (cl-tf2:lookup-transform tf frame-id child-frame-id)
    (cl-tf2::tf2-server-error () (progn (sleep 0.1) (tf2-lookup tf frame-id child-frame-id)))))

(defun tf2-transform-pose (tf pose frame-id target-frame)
  (declare (type cl-tf2:buffer-client tf)
           (type cl-transforms:pose pose)
           (type string frame-id target-frame))
  (alexandria:when-let*
      ((transform-stamped (tf2-lookup tf target-frame frame-id))
       (transform (cl-tf2:transform transform-stamped)))
    (cl-transforms:transform-pose transform pose)))

(defun tf2-transform-pose-stamped-msg (tf pose-stamped-msg target-frame)
  (with-fields ((frame-id (frame_id header)) pose) pose-stamped-msg
    (tf2-transform-pose tf (pose-msg->pose pose) frame-id target-frame)))
  
(defun publish-tool-poses-to-tf (demo-handle desigs)
  (alexandria:when-let ((transforms
                         (remove-if-not
                          #'identity
                          (loop for desig in desigs
                                counting t into index
                                collect (infer-object-transform desig (write-to-string index)))))
                        (broadcaster (getf demo-handle :tf-broadcaster)))
    (publish broadcaster (make-message "tf2_msgs/TFMessage" :transforms (coerce transforms 'vector)))))
