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
  :slots (_currentX _currentY _ok ))

(defmethod assignment1::GoToResponse
  (:init
   (&key
    ((:currentX __currentX) 0)
    ((:currentY __currentY) 0)
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _currentX (round __currentX))
   (setq _currentY (round __currentY))
   (setq _ok __ok)
   self)
  (:currentX
   (&optional __currentX)
   (if __currentX (setq _currentX __currentX)) _currentX)
  (:currentY
   (&optional __currentY)
   (if __currentY (setq _currentY __currentY)) _currentY)
  (:ok
   (&optional __ok)
   (if __ok (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; int64 _currentX
    8
    ;; int64 _currentY
    8
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _currentX
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _currentX (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _currentX) (= (length (_currentX . bv)) 2)) ;; bignum
              (write-long (ash (elt (_currentX . bv) 0) 0) s)
              (write-long (ash (elt (_currentX . bv) 1) -1) s))
             ((and (class _currentX) (= (length (_currentX . bv)) 1)) ;; big1
              (write-long (elt (_currentX . bv) 0) s)
              (write-long (if (>= _currentX 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _currentX s)(write-long (if (>= _currentX 0) 0 #xffffffff) s)))
     ;; int64 _currentY
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _currentY (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _currentY) (= (length (_currentY . bv)) 2)) ;; bignum
              (write-long (ash (elt (_currentY . bv) 0) 0) s)
              (write-long (ash (elt (_currentY . bv) 1) -1) s))
             ((and (class _currentY) (= (length (_currentY . bv)) 1)) ;; big1
              (write-long (elt (_currentY . bv) 0) s)
              (write-long (if (>= _currentY 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _currentY s)(write-long (if (>= _currentY 0) 0 #xffffffff) s)))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _currentX
#+(or :alpha :irix6 :x86_64)
      (setf _currentX (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _currentX (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _currentY
#+(or :alpha :irix6 :x86_64)
      (setf _currentY (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _currentY (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass assignment1::GoTo
  :super ros::object
  :slots ())

(setf (get assignment1::GoTo :md5sum-) "5ec21d6e53db8f56a8468d99e5c23c5b")
(setf (get assignment1::GoTo :datatype-) "assignment1/GoTo")
(setf (get assignment1::GoTo :request) assignment1::GoToRequest)
(setf (get assignment1::GoTo :response) assignment1::GoToResponse)

(defmethod assignment1::GoToRequest
  (:response () (instance assignment1::GoToResponse :init)))

(setf (get assignment1::GoToRequest :md5sum-) "5ec21d6e53db8f56a8468d99e5c23c5b")
(setf (get assignment1::GoToRequest :datatype-) "assignment1/GoToRequest")
(setf (get assignment1::GoToRequest :definition-)
      "int64 x
int64 y
---
int64 currentX
int64 currentY
bool ok

")

(setf (get assignment1::GoToResponse :md5sum-) "5ec21d6e53db8f56a8468d99e5c23c5b")
(setf (get assignment1::GoToResponse :datatype-) "assignment1/GoToResponse")
(setf (get assignment1::GoToResponse :definition-)
      "int64 x
int64 y
---
int64 currentX
int64 currentY
bool ok

")



(provide :assignment1/GoTo "5ec21d6e53db8f56a8468d99e5c23c5b")


