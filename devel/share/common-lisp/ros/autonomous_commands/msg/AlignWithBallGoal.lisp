; Auto-generated. Do not edit!


(cl:in-package autonomous_commands-msg)


;//! \htmlinclude AlignWithBallGoal.msg.html

(cl:defclass <AlignWithBallGoal> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AlignWithBallGoal (<AlignWithBallGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignWithBallGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignWithBallGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomous_commands-msg:<AlignWithBallGoal> is deprecated: use autonomous_commands-msg:AlignWithBallGoal instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignWithBallGoal>) ostream)
  "Serializes a message object of type '<AlignWithBallGoal>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignWithBallGoal>) istream)
  "Deserializes a message object of type '<AlignWithBallGoal>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignWithBallGoal>)))
  "Returns string type for a message object of type '<AlignWithBallGoal>"
  "autonomous_commands/AlignWithBallGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignWithBallGoal)))
  "Returns string type for a message object of type 'AlignWithBallGoal"
  "autonomous_commands/AlignWithBallGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignWithBallGoal>)))
  "Returns md5sum for a message object of type '<AlignWithBallGoal>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignWithBallGoal)))
  "Returns md5sum for a message object of type 'AlignWithBallGoal"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignWithBallGoal>)))
  "Returns full string definition for message of type '<AlignWithBallGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignWithBallGoal)))
  "Returns full string definition for message of type 'AlignWithBallGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignWithBallGoal>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignWithBallGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignWithBallGoal
))