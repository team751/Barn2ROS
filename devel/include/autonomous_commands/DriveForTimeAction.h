// Generated by gencpp from file autonomous_commands/DriveForTimeAction.msg
// DO NOT EDIT!


#ifndef AUTONOMOUS_COMMANDS_MESSAGE_DRIVEFORTIMEACTION_H
#define AUTONOMOUS_COMMANDS_MESSAGE_DRIVEFORTIMEACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <autonomous_commands/DriveForTimeActionGoal.h>
#include <autonomous_commands/DriveForTimeActionResult.h>
#include <autonomous_commands/DriveForTimeActionFeedback.h>

namespace autonomous_commands
{
template <class ContainerAllocator>
struct DriveForTimeAction_
{
  typedef DriveForTimeAction_<ContainerAllocator> Type;

  DriveForTimeAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  DriveForTimeAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
    }



   typedef  ::autonomous_commands::DriveForTimeActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::autonomous_commands::DriveForTimeActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::autonomous_commands::DriveForTimeActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;




  typedef boost::shared_ptr< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> const> ConstPtr;

}; // struct DriveForTimeAction_

typedef ::autonomous_commands::DriveForTimeAction_<std::allocator<void> > DriveForTimeAction;

typedef boost::shared_ptr< ::autonomous_commands::DriveForTimeAction > DriveForTimeActionPtr;
typedef boost::shared_ptr< ::autonomous_commands::DriveForTimeAction const> DriveForTimeActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autonomous_commands

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'autonomous_commands': ['/home/sam/Barn2ROS/devel/share/autonomous_commands/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dac433e38854be176dee5c31677cd989";
  }

  static const char* value(const ::autonomous_commands::DriveForTimeAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdac433e38854be17ULL;
  static const uint64_t static_value2 = 0x6dee5c31677cd989ULL;
};

template<class ContainerAllocator>
struct DataType< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autonomous_commands/DriveForTimeAction";
  }

  static const char* value(const ::autonomous_commands::DriveForTimeAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
DriveForTimeActionGoal action_goal\n\
DriveForTimeActionResult action_result\n\
DriveForTimeActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
DriveForTimeGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define the goal\n\
geometry_msgs/Twist cmd\n\
int64 time\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
DriveForTimeResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define the result\n\
\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
DriveForTimeFeedback feedback\n\
\n\
================================================================================\n\
MSG: autonomous_commands/DriveForTimeFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define a feedback message\n\
int64 time\n\
";
  }

  static const char* value(const ::autonomous_commands::DriveForTimeAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DriveForTimeAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autonomous_commands::DriveForTimeAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autonomous_commands::DriveForTimeAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::autonomous_commands::DriveForTimeActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::autonomous_commands::DriveForTimeActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::autonomous_commands::DriveForTimeActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTONOMOUS_COMMANDS_MESSAGE_DRIVEFORTIMEACTION_H
