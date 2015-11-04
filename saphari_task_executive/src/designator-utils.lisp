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

(defun action-designator (descr)
  "Creates an action designator with description 'descr'."
  (desig:make-designator 'desig:action descr))

(defun location-designator (descr)
  "Creates a location designator with description 'descr'."
  (desig:make-designator 'desig:location descr))

(defun object-designator (descr)
  "Creates an object designator with description 'descr'."
  (desig:make-designator 'desig:object descr))

(defun desig-p (desig)
  "Checks whether 'desig' is an of type designator object."
  (typep desig 'desig:designator))

(defun desig-prop-value (desig prop)
  "Returns the value associated with the property 'prop' in
 the description of 'desig'. Returns NIL if either 'desig'
 is not a designator, or 'prop' is not part of its description."
  (when (desig-p desig)
    (desig:desig-prop-value desig prop)))

(defun desig-prop-value-p (desig prop value &key (test #'equal))
  "Checks whether designator 'desig' has property 'prop'
 associated with 'value' in its description. Return NIL if
 'desig' is not a designator. Optionally, the test function
 can be changed through the keyword parameter :test."
  (funcall test (desig-prop-value desig prop) value))

(defun desig-descr-equal (desig-1 desig-2)
  "Checks whether the two designators 'desig-1' and 'desig-2'
 have descriptions that are equal. Also works if either or both
 of their descriptions contain nested designators. Returns NIL if
 either of the two inputs is not an instance of type designator."
  (and
   (desig-p desig-1)
   (desig-p desig-2)
   (every #'identity
          (mapcar (lambda (prop-value-pair)
                    (destructuring-bind (prop value) prop-value-pair
                      (desig-prop-value-p
                       desig-2 prop value
                       :test (if (desig-p value) #'desig-descr-equal #'equal))))
                  (desig:description desig-1)))))
