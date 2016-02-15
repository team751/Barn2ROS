// Generated by gencpp from file autonomous_commands/DriveToDefenseFeedback.msg
// DO NOT EDIT!


#ifndef AUTONOMOUS_COMMANDS_MESSAGE_DRIVETODEFENSEFEEDBACK_H
#define AUTONOMOUS_COMMANDS_MESSAGE_DRIVETODEFENSEFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace autonomous_commands
{
template <class ContainerAllocator>
struct DriveToDefenseFeedback_
{
  typedef DriveToDefenseFeedback_<ContainerAllocator> Type;

  DriveToDefenseFeedback_()
    {
    }
  DriveToDefenseFeedback_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct DriveToDefenseFeedback_

typedef ::autonomous_commands::DriveToDefenseFeedback_<std::allocator<void> > DriveToDefenseFeedback;

typedef boost::shared_ptr< ::autonomous_commands::DriveToDefenseFeedback > DriveToDefenseFeedbackPtr;
typedef boost::shared_ptr< ::autonomous_commands::DriveToDefenseFeedback const> DriveToDefenseFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autonomous_commands

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'autonomous_commands': ['/home/sam/Barn2ROS/devel/share/autonomous_commands/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autonomous_commands/DriveToDefenseFeedback";
  }

  static const char* value(const ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define a feedback message\n\
";
  }

  static const char* value(const ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DriveToDefenseFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::autonomous_commands::DriveToDefenseFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // AUTONOMOUS_COMMANDS_MESSAGE_DRIVETODEFENSEFEEDBACK_H
