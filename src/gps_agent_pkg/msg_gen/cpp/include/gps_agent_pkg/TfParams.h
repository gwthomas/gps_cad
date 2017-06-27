/* Auto-generated by genmsg_cpp for file /home/melissachien/new_gps/src/gps_agent_pkg/msg/TfParams.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_TFPARAMS_H
#define GPS_AGENT_PKG_MESSAGE_TFPARAMS_H
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
struct TfParams_ {
  typedef TfParams_<ContainerAllocator> Type;

  TfParams_()
  : dU(0)
  {
  }

  TfParams_(const ContainerAllocator& _alloc)
  : dU(0)
  {
  }

  typedef uint32_t _dU_type;
  uint32_t dU;


  typedef boost::shared_ptr< ::gps_agent_pkg::TfParams_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::TfParams_<ContainerAllocator>  const> ConstPtr;
}; // struct TfParams
typedef  ::gps_agent_pkg::TfParams_<std::allocator<void> > TfParams;

typedef boost::shared_ptr< ::gps_agent_pkg::TfParams> TfParamsPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::TfParams const> TfParamsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::TfParams_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::TfParams_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfParams_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfParams_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::TfParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "24131aa271a6e7d452bc4f62ea3b4c2b";
  }

  static const char* value(const  ::gps_agent_pkg::TfParams_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x24131aa271a6e7d4ULL;
  static const uint64_t static_value2 = 0x52bc4f62ea3b4c2bULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::TfParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/TfParams";
  }

  static const char* value(const  ::gps_agent_pkg::TfParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::TfParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Tf Params. just need to track dU.\n\
uint32 dU\n\
\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::TfParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::gps_agent_pkg::TfParams_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::TfParams_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.dU);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TfParams_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::TfParams_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::TfParams_<ContainerAllocator> & v) 
  {
    s << indent << "dU: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.dU);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_TFPARAMS_H

