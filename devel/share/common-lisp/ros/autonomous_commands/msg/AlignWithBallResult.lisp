; Auto-generated. Do not edit!


(cl:in-package autonomous_commands-msg)


;//! \htmlinclude AlignWithBallResult.msg.html

(cl:defclass <AlignWithBallResult> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AlignWithBallResult (<AlignWithBallResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignWithBallResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignWithBallResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomous_commands-msg:<AlignWithBallResult> is deprecated: use autonomous_commands-msg:AlignWithBallResult instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignWithBallResult>) ostream)
  "Serializes a message object of type '<AlignWithBallResult>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignWithBallResult>) istream)
  "Deserializes a message object of type '<AlignWithBallResult>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignWithBallResult>)))
  "Returns string type for a message object of type '<AlignWithBallResult>"
  "autonomous_commands/AlignWithBallResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignWithBallResult)))
  "Returns string type for a message object of type 'AlignWithBallResult"
  "autonomous_commands/AlignWithBallResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignWithBallResult>)))
  "Returns md5sum for a message object of type '<AlignWithBallResult>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignWithBallResult)))
  "Returns md5sum for a message object of type 'AlignWithBallResult"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignWithBallResult>)))
  "Returns full string definition for message of type '<AlignWithBallResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignWithBallResult)))
  "Returns full string definition for message of type 'AlignWithBallResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignWithBallResult>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignWithBallResult>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignWithBallResult
))
