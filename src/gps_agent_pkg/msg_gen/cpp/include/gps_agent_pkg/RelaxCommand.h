/* Auto-generated by genmsg_cpp for file /home/melissachien/new_gps/src/gps_agent_pkg/msg/RelaxCommand.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_RELAXCOMMAND_H
#define GPS_AGENT_PKG_MESSAGE_RELAXCOMMAND_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace gps_agent_pkg
{
template <class ContainerAllocator>
struct RelaxCommand_ {
  typedef RelaxCommand_<ContainerAllocator> Type;

  RelaxCommand_()
  : stamp()
  , id(0)
  , arm(0)
  {
  }

  RelaxCommand_(const ContainerAllocator& _alloc)
  : stamp()
  , id(0)
  , arm(0)
  {
  }

  typedef ros::Time _stamp_type;
  ros::Time stamp;

  typedef int32_t _id_type;
  int32_t id;

  typedef int32_t _arm_type;
  int32_t arm;


  typedef boost::shared_ptr< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator>  const> ConstPtr;
}; // struct RelaxCommand
typedef  ::gps_agent_pkg::RelaxCommand_<std::allocator<void> > RelaxCommand;

typedef boost::shared_ptr< ::gps_agent_pkg::RelaxCommand> RelaxCommandPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::RelaxCommand const> RelaxCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "08fa438768d567bd060ce1ef6e4edc87";
  }

  static const char* value(const  ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x08fa438768d567bdULL;
  static const uint64_t static_value2 = 0x060ce1ef6e4edc87ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/RelaxCommand";
  }

  static const char* value(const  ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "time stamp\n\
int32 id  # ID must be echoed back in SampleResult\n\
int32 arm  # which arm to relax (ActuatorType enum)\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.stamp);
    stream.next(m.id);
    stream.next(m.arm);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RelaxCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::RelaxCommand_<ContainerAllocator> & v) 
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "arm: ";
    Printer<int32_t>::stream(s, indent + "  ", v.arm);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_RELAXCOMMAND_H

