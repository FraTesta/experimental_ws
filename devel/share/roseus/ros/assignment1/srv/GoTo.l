;; Auto-generated. Do not edit!


(when (boundp 'assignment1::GoTo)
  (if (not (find-package "ASSIGNMENT1"))
    (make-package "ASSIGNMENT1"))
  (shadow 'GoTo (find-package "ASSIGNMENT1")))
(unless (find-package "ASSIGNMENT1::GOTO")
  (make-package "ASSIGNMENT1::GOTO"))
(unless (find-package "ASSIGNMENT1::GOTOREQUEST")
  (make-package "ASSIGNMENT1::GOTOREQUEST"))
(unless (find-package "ASSIGNMENT1::GOTORESPONSE")
  (make-package "ASSIGNMENT1::GOTORESPONSE"))

(in-package "ROS")





(defclass assignment1::GoToRequest
  :super ros::object
  :slots (_x _y ))

(defmethod assignment1::GoToRequest
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; int64 _x
    8
    ;; int64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _x
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x) (= (length (_x . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x . bv) 0) 0) s)
              (write-long (ash (elt (_x . bv) 1) -1) s))
             ((and (class _x) (= (length (_x . bv)) 1)) ;; big1
              (write-long (elt (_x . bv) 0) s)
              (write-long (if (>= _x 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x s)(write-long (if (>= _x 0) 0 #xffffffff) s)))
     ;; int64 _y
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y) (= (length (_y . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y . bv) 0) 0) s)
              (write-long (ash (elt (_y . bv) 1) -1) s))
             ((and (class _y) (= (length (_y . bv)) 1)) ;; big1
              (write-long (elt (_y . bv) 0) s)
              (write-long (if (>= _y 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y s)(write-long (if (>= _y 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _x
#+(or :alpha :irix6 :x86_64)
      (setf _x (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y
#+(or :alpha :irix6 :x86_64)
      (setf _y (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass assignment1::GoToResponse
  :super ros::object
  :slots (_o ))

(defmethod assignment1::GoToResponse
  (:init
   (&key
    ((:o __o) 0)
    )
   (send-super :init)
   (setq _o (round __o))
   self)
  (:o
   (&optional __o)
   (if __o (setq _o __o)) _o)
  (:serialization-length
   ()
   (+
    ;; int64 _o
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _o
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _o (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _o) (= (length (_o . bv)) 2)) ;; bignum
              (write-long (ash (elt (_o . bv) 0) 0) s)
              (write-long (ash (elt (_o . bv) 1) -1) s))
             ((and (class _o) (= (length (_o . bv)) 1)) ;; big1
              (write-long (elt (_o . bv) 0) s)
              (write-long (if (>= _o 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _o s)(write-long (if (>= _o 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _o
#+(or :alpha :irix6 :x86_64)
      (setf _o (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _o (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass assignment1::GoTo
  :super ros::object
  :slots ())

(setf (get assignment1::GoTo :md5sum-) "6835bdfd1be90305ffbc4f2fb8805255")
(setf (get assignment1::GoTo :datatype-) "assignment1/GoTo")
(setf (get assignment1::GoTo :request) assignment1::GoToRequest)
(setf (get assignment1::GoTo :response) assignment1::GoToResponse)

(defmethod assignment1::GoToRequest
  (:response () (instance assignment1::GoToResponse :init)))

(setf (get assignment1::GoToRequest :md5sum-) "6835bdfd1be90305ffbc4f2fb8805255")
(setf (get assignment1::GoToRequest :datatype-) "assignment1/GoToRequest")
(setf (get assignment1::GoToRequest :definition-)
      "int64 x
int64 y
---
int64 o

")

(setf (get assignment1::GoToResponse :md5sum-) "6835bdfd1be90305ffbc4f2fb8805255")
(setf (get assignment1::GoToResponse :datatype-) "assignment1/GoToResponse")
(setf (get assignment1::GoToResponse :definition-)
      "int64 x
int64 y
---
int64 o

")



(provide :assignment1/GoTo "6835bdfd1be90305ffbc4f2fb8805255")


