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
;;; STRING UTILS
;;;

(defun conc-strings (&rest strings)
  (apply #'concatenate 'string strings))
(defun string->keyword (s)
  (declare (type string s))
  (intern (string-upcase s) :keyword))

;;;
;;; LIST UTILS
;;;

(defun conc-lists (&rest lists)
  (apply #'concatenate 'list lists))

(defun get-duplicates (sequence &optional (singletons nil))
  "Returns the list of duplicate elements in 'sequence', i.e. elements that occur
 more than once in 'sequence'. Optionally, one can specify a list of elements
 'singletons' which shall already be counted as duplicates after the first occurence."
  ;;; recursion with implicit anchor of nil for an empty sequence
  (when sequence
    (if (member (car sequence) singletons)
        ;;; the first element in sequence is a duplicate
        (let ((duplicates (get-duplicates (rest sequence) singletons)))
          (if (member (car sequence) duplicates)
              ;;; the first element in sequence shows up later in the sequence, too
              duplicates
              ;;; the first element in sequence has not yet been counted in duplicates
              (cons (car sequence) duplicates)))
        ;;; the first element in sequence is not a duplicate
        (get-duplicates (rest sequence) (cons (car sequence) singletons)))))

;;;
;;; KEYWORD-PLIST UTILS
;;;
      
(defun keyword-plist-p (my-plist)
  "Checks whether 'my-plist' is a proper plist with only keywords as keys."
  (declare (type list my-plist))
  (if my-plist
      ;;; recursion
      (and
       (evenp (length my-plist))
       (destructuring-bind (key value &rest rest) my-plist
         (declare (ignore value))
         (and (keywordp key) (keyword-plist-p rest))))
      ;;; anchor
      t))

(defun ensure-keyword-plist (my-plist)
  "Throws an error if 'my-plist' is not a proper plist with only
 keywords as keys."
  (unless (keyword-plist-p my-plist)
    (error "~a is not a plist with only keys as keywords." my-plist)))

(defun plist-keys (my-plist)
  "Returns the keys of plist my-plist, assuming that all keys are keywords.

   NOTE: my-plist will remain unchanged."
  (remove-if-not #'keywordp my-plist))

(defun merge-keyword-plists (&rest plists)
  "Merges the keyword-plists 'plist' under the assumption that they do not
 hold duplicate keys. Throws an error if any of the inputs is not such a
 keyword-plist, or if there are duplicate keys between the 'plists'."
  (mapcar #'ensure-keyword-plist plists)
  (alexandria:when-let ((duplicate-keys (get-duplicates (apply #'conc-lists (mapcar #'plist-keys plists)))))
    (error "Merge-plists encountered the following duplicate keys in its inputs: ~a" duplicate-keys))
  (apply #'conc-lists plists))

(defun plist->list-of-lists (plist)
  "Recursively turns 'plist' into a list of lists."
  (if (and (listp plist) (evenp (length plist)))
      (if (not plist)
          nil
          (destructuring-bind (key value &rest rest) plist
            (conc-lists
             (list (list key (if (and (listp value) (keyword-plist-p value))
                                 (plist->list-of-lists value)
                                 value)))
             (plist->list-of-lists rest))))))
      
;;;
;;; ALIST UTILS
;;;

(defun remove-from-alist (key alist &key (test #'equal))
  (remove-if (alexandria:curry test key) alist :key #'car))

(defun set-in-alist (key datum alist)
  (acons key datum (remove-from-alist key alist)))
