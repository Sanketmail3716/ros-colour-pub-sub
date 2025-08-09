;; Auto-generated. Do not edit!


(when (boundp 'custom::colour)
  (if (not (find-package "CUSTOM"))
    (make-package "CUSTOM"))
  (shadow 'colour (find-package "CUSTOM")))
(unless (find-package "CUSTOM::COLOUR")
  (make-package "CUSTOM::COLOUR"))

(in-package "ROS")
;;//! \htmlinclude colour.msg.html


(defclass custom::colour
  :super ros::object
  :slots (_r _g _b _a _name ))

(defmethod custom::colour
  (:init
   (&key
    ((:r __r) 0)
    ((:g __g) 0)
    ((:b __b) 0)
    ((:a __a) 0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _r (round __r))
   (setq _g (round __g))
   (setq _b (round __b))
   (setq _a (round __a))
   (setq _name (string __name))
   self)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; int64 _r
    8
    ;; int64 _g
    8
    ;; int64 _b
    8
    ;; int64 _a
    8
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _r
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _r (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _r) (= (length (_r . bv)) 2)) ;; bignum
              (write-long (ash (elt (_r . bv) 0) 0) s)
              (write-long (ash (elt (_r . bv) 1) -1) s))
             ((and (class _r) (= (length (_r . bv)) 1)) ;; big1
              (write-long (elt (_r . bv) 0) s)
              (write-long (if (>= _r 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _r s)(write-long (if (>= _r 0) 0 #xffffffff) s)))
     ;; int64 _g
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _g (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _g) (= (length (_g . bv)) 2)) ;; bignum
              (write-long (ash (elt (_g . bv) 0) 0) s)
              (write-long (ash (elt (_g . bv) 1) -1) s))
             ((and (class _g) (= (length (_g . bv)) 1)) ;; big1
              (write-long (elt (_g . bv) 0) s)
              (write-long (if (>= _g 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _g s)(write-long (if (>= _g 0) 0 #xffffffff) s)))
     ;; int64 _b
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _b (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _b) (= (length (_b . bv)) 2)) ;; bignum
              (write-long (ash (elt (_b . bv) 0) 0) s)
              (write-long (ash (elt (_b . bv) 1) -1) s))
             ((and (class _b) (= (length (_b . bv)) 1)) ;; big1
              (write-long (elt (_b . bv) 0) s)
              (write-long (if (>= _b 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _b s)(write-long (if (>= _b 0) 0 #xffffffff) s)))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _r
#+(or :alpha :irix6 :x86_64)
      (setf _r (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _r (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _g
#+(or :alpha :irix6 :x86_64)
      (setf _g (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _g (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _b
#+(or :alpha :irix6 :x86_64)
      (setf _b (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _b (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get custom::colour :md5sum-) "348ffe1626e833387879188565686257")
(setf (get custom::colour :datatype-) "custom/colour")
(setf (get custom::colour :definition-)
      "int64 r
int64 g
int64 b
int64 a
string name

")



(provide :custom/colour "348ffe1626e833387879188565686257")


