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

(asdf:defsystem saphari-task-executive
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :license "BSD"
  :description "Task executive of the final review demo of the SAPHARI project."
  :depends-on (roslisp
               cram-json-prolog
               cram-beliefstate
               roslisp-beasty
               cram-dlr-wsg50
               designators
               cram-language-designator-support
               saphari_tool_detector-srv
               visualization_msgs-msg
               saphari_msgs-msg)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "lisp-utils" :depends-on ("package"))
     (:file "knowrob-utils" :depends-on ("package"))
     (:file "designator-utils" :depends-on ("package"))
     (:file "logging-utils" :depends-on ("package"))
     (:file "conversions" :depends-on ("package" "lisp-utils" "designator-utils"))
     (:file "tf" :depends-on ("package" "conversions"))
     (:file "knowrob-queries" :depends-on ("package" "knowrob-utils" "conversions"))
     (:file "arm-control" :depends-on ("package" "lisp-utils" "conversions"))
     (:file "designator-reasoning" :depends-on ("package" "designator-utils" "tf" "arm-control"))
     (:file "marker-viz" :depends-on ("package" "designator-reasoning" "designator-utils"))
     (:file "tool-perception" :depends-on ("package" "lisp-utils" "knowrob-utils" "designator-reasoning" "conversions" "marker-viz"))
     (:file "human-perception" :depends-on ("package" "lisp-utils" "designator-utils" "conversions" "marker-viz" "tf"))
     (:file "plans" :depends-on ("package" "logging-utils" "designator-utils" "designator-reasoning" "tool-perception" "arm-control" "marker-viz" "human-perception"))
     (:file "main" :depends-on ("package" "tool-perception" "arm-control" "plans"))))))
