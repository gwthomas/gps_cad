/* Auto-generated by genmsg_cpp for file /home/gwthomas/workspace/gps/src/gps_agent_pkg/msg/TfObsData.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_TFOBSDATA_H
#define GPS_AGENT_PKG_MESSAGE_TFOBSDATA_H
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
struct TfObsData_ {
  typedef TfObsData_<ContainerAllocator> Type;

  TfObsData_()
  : data()
  , shape()
  {
  }

  TfObsData_(const ContainerAllocator& _alloc)
  : data(_alloc)
  , shape(_alloc)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  data;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _shape_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  shape;


  typedef boost::shared_ptr< ::gps_agent_pkg::TfObsData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::TfObsData_<ContainerAllocator>  const> ConstPtr;
}; // struct TfObsData
typedef  ::gps_agent_pkg::TfObsData_<std::allocator<void> > TfObsData;

typedef boost::shared_ptr< ::gps_agent_pkg::TfObsData> TfObsDataPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::TfObsData const> TfObsDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::TfObsData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::TfObsData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfObsData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfObsData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::TfObsData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1c5d3c77f5a9734934108f61a36f5512";
  }

  static const char* value(const  ::gps_agent_pkg::TfObsData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1c5d3c77f5a97349ULL;
  static const uint64_t static_value2 = 0x34108f61a36f5512ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::TfObsData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/TfObsData";
  }

  static const char* value(const  ::gps_agent_pkg::TfObsData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::TfObsData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] data\n\
int32[] shape\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::TfObsData_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::TfObsData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
    stream.next(m.shape);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TfObsData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::TfObsData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::TfObsData_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "shape[]" << std::endl;
    for (size_t i = 0; i < v.shape.size(); ++i)
    {
      s << indent << "  shape[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.shape[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_TFOBSDATA_H

