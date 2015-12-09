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

(defun beasty-default-parameters ()
  "Returns the default parameters of beasty as a list in the following order:
 ACTION-NAME, USER-ID, and USER-PASSWORD."
  (list "beasty" 1 1337))

(defun make-and-init-beasty-handle (&optional (sim-p t))
  "Creates, initialises and returns a beasty handle using the default parameters."
  (apply #'roslisp-beasty:make-and-init-beasty-handle (conc-lists (beasty-default-parameters) (list sim-p))))

(defun make-wsg-handle ()
  (cram-dlr-wsg50:make-dlr-wsg50-handle "EDDIE/gripper"))

(defun saphari-tool-config ()
  (roslisp-beasty:tool-config :tool-mass 1.58 :tool-com  (make-3d-vector 0 0 0.17)))

(defun joint-goal (goal-config sim-p)
  (roslisp-beasty:goal
   :command-config (roslisp-beasty:command-config :command-type :joint-impedance)
   :joint-goal-config (roslisp-beasty:joint-goal-config :joint-goal-config goal-config)
   :simulated-config (roslisp-beasty:simulated-config :simulated-robot sim-p)
   :tool-config (saphari-tool-config)
   :motor-power-config (roslisp-beasty:motor-power-config :motor-power t)))

(defun cartesian-goal (goal-pose sim-p)
  (roslisp-beasty:goal
   :command-config (roslisp-beasty:command-config :command-type :cartesian-impedance)
   :cartesian-goal-config (roslisp-beasty:cartesian-goal-config :cartesian-goal-pose goal-pose)
   :simulated-config (roslisp-beasty:simulated-config :simulated-robot sim-p)
   :tool-config (saphari-tool-config)
   :motor-power-config (roslisp-beasty:motor-power-config :motor-power t)))
   
