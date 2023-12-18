/* Auto-generated by genmsg_cpp for file /home/jchen/workspace/src/quadrotor_msgs/msg/PPROutputData.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_PPROUTPUTDATA_H
#define QUADROTOR_MSGS_MESSAGE_PPROUTPUTDATA_H
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

#include "std_msgs/Header.h"

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct PPROutputData_ {
  typedef PPROutputData_<ContainerAllocator> Type;

  PPROutputData_()
  : header()
  , quad_time(0)
  , des_thrust(0.0)
  , des_roll(0.0)
  , des_pitch(0.0)
  , des_yaw(0.0)
  , est_roll(0.0)
  , est_pitch(0.0)
  , est_yaw(0.0)
  , est_angvel_x(0.0)
  , est_angvel_y(0.0)
  , est_angvel_z(0.0)
  , est_acc_x(0.0)
  , est_acc_y(0.0)
  , est_acc_z(0.0)
  , pwm()
  {
    pwm.assign(0);
  }

  PPROutputData_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , quad_time(0)
  , des_thrust(0.0)
  , des_roll(0.0)
  , des_pitch(0.0)
  , des_yaw(0.0)
  , est_roll(0.0)
  , est_pitch(0.0)
  , est_yaw(0.0)
  , est_angvel_x(0.0)
  , est_angvel_y(0.0)
  , est_angvel_z(0.0)
  , est_acc_x(0.0)
  , est_acc_y(0.0)
  , est_acc_z(0.0)
  , pwm()
  {
    pwm.assign(0);
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint16_t _quad_time_type;
  uint16_t quad_time;

  typedef double _des_thrust_type;
  double des_thrust;

  typedef double _des_roll_type;
  double des_roll;

  typedef double _des_pitch_type;
  double des_pitch;

  typedef double _des_yaw_type;
  double des_yaw;

  typedef double _est_roll_type;
  double est_roll;

  typedef double _est_pitch_type;
  double est_pitch;

  typedef double _est_yaw_type;
  double est_yaw;

  typedef double _est_angvel_x_type;
  double est_angvel_x;

  typedef double _est_angvel_y_type;
  double est_angvel_y;

  typedef double _est_angvel_z_type;
  double est_angvel_z;

  typedef double _est_acc_x_type;
  double est_acc_x;

  typedef double _est_acc_y_type;
  double est_acc_y;

  typedef double _est_acc_z_type;
  double est_acc_z;

  typedef boost::array<uint16_t, 4>  _pwm_type;
  boost::array<uint16_t, 4>  pwm;


  typedef boost::shared_ptr< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::PPROutputData_<ContainerAllocator>  const> ConstPtr;
}; // struct PPROutputData
typedef  ::quadrotor_msgs::PPROutputData_<std::allocator<void> > PPROutputData;

typedef boost::shared_ptr< ::quadrotor_msgs::PPROutputData> PPROutputDataPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::PPROutputData const> PPROutputDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::PPROutputData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PPROutputData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "732c0e3ca36f241464f8c445e78a0d0a";
  }

  static const char* value(const  ::quadrotor_msgs::PPROutputData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x732c0e3ca36f2414ULL;
  static const uint64_t static_value2 = 0x64f8c445e78a0d0aULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/PPROutputData";
  }

  static const char* value(const  ::quadrotor_msgs::PPROutputData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
uint16 quad_time\n\
float64 des_thrust\n\
float64 des_roll\n\
float64 des_pitch\n\
float64 des_yaw\n\
float64 est_roll\n\
float64 est_pitch\n\
float64 est_yaw\n\
float64 est_angvel_x\n\
float64 est_angvel_y\n\
float64 est_angvel_z\n\
float64 est_acc_x\n\
float64 est_acc_y\n\
float64 est_acc_z\n\
uint16[4] pwm\n\
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
";
  }

  static const char* value(const  ::quadrotor_msgs::PPROutputData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quadrotor_msgs::PPROutputData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.quad_time);
    stream.next(m.des_thrust);
    stream.next(m.des_roll);
    stream.next(m.des_pitch);
    stream.next(m.des_yaw);
    stream.next(m.est_roll);
    stream.next(m.est_pitch);
    stream.next(m.est_yaw);
    stream.next(m.est_angvel_x);
    stream.next(m.est_angvel_y);
    stream.next(m.est_angvel_z);
    stream.next(m.est_acc_x);
    stream.next(m.est_acc_y);
    stream.next(m.est_acc_z);
    stream.next(m.pwm);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PPROutputData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::PPROutputData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::PPROutputData_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "quad_time: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.quad_time);
    s << indent << "des_thrust: ";
    Printer<double>::stream(s, indent + "  ", v.des_thrust);
    s << indent << "des_roll: ";
    Printer<double>::stream(s, indent + "  ", v.des_roll);
    s << indent << "des_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.des_pitch);
    s << indent << "des_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.des_yaw);
    s << indent << "est_roll: ";
    Printer<double>::stream(s, indent + "  ", v.est_roll);
    s << indent << "est_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.est_pitch);
    s << indent << "est_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.est_yaw);
    s << indent << "est_angvel_x: ";
    Printer<double>::stream(s, indent + "  ", v.est_angvel_x);
    s << indent << "est_angvel_y: ";
    Printer<double>::stream(s, indent + "  ", v.est_angvel_y);
    s << indent << "est_angvel_z: ";
    Printer<double>::stream(s, indent + "  ", v.est_angvel_z);
    s << indent << "est_acc_x: ";
    Printer<double>::stream(s, indent + "  ", v.est_acc_x);
    s << indent << "est_acc_y: ";
    Printer<double>::stream(s, indent + "  ", v.est_acc_y);
    s << indent << "est_acc_z: ";
    Printer<double>::stream(s, indent + "  ", v.est_acc_z);
    s << indent << "pwm[]" << std::endl;
    for (size_t i = 0; i < v.pwm.size(); ++i)
    {
      s << indent << "  pwm[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.pwm[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_PPROUTPUTDATA_H

