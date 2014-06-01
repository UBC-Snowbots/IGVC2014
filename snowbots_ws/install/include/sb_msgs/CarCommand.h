/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/jannicke/catkin_ws/src/sb_msgs/msg/CarCommand.msg
 *
 */


#ifndef SB_MSGS_MESSAGE_CARCOMMAND_H
#define SB_MSGS_MESSAGE_CARCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sb_msgs
{
template <class ContainerAllocator>
struct CarCommand_
{
  typedef CarCommand_<ContainerAllocator> Type;

  CarCommand_()
    : throttle(0.0)
    , steering(0.0)  {
    }
  CarCommand_(const ContainerAllocator& _alloc)
    : throttle(0.0)
    , steering(0.0)  {
    }



   typedef double _throttle_type;
  _throttle_type throttle;

   typedef double _steering_type;
  _steering_type steering;




  typedef boost::shared_ptr< ::sb_msgs::CarCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sb_msgs::CarCommand_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct CarCommand_

typedef ::sb_msgs::CarCommand_<std::allocator<void> > CarCommand;

typedef boost::shared_ptr< ::sb_msgs::CarCommand > CarCommandPtr;
typedef boost::shared_ptr< ::sb_msgs::CarCommand const> CarCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sb_msgs::CarCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sb_msgs::CarCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sb_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sb_msgs': ['/home/jannicke/catkin_ws/src/sb_msgs/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sb_msgs::CarCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sb_msgs::CarCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sb_msgs::CarCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sb_msgs::CarCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sb_msgs::CarCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sb_msgs::CarCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sb_msgs::CarCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39f463d271c2ca10c14182802c72c029";
  }

  static const char* value(const ::sb_msgs::CarCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39f463d271c2ca10ULL;
  static const uint64_t static_value2 = 0xc14182802c72c029ULL;
};

template<class ContainerAllocator>
struct DataType< ::sb_msgs::CarCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sb_msgs/CarCommand";
  }

  static const char* value(const ::sb_msgs::CarCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sb_msgs::CarCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Car command that arduinoDriver or furiousDriver can understand\n\
\n\
float64 throttle # throttle value -1 < x < 1\n\
float64 steering # steer value -1 < x < 1\n\
";
  }

  static const char* value(const ::sb_msgs::CarCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sb_msgs::CarCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.throttle);
      stream.next(m.steering);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CarCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sb_msgs::CarCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sb_msgs::CarCommand_<ContainerAllocator>& v)
  {
    s << indent << "throttle: ";
    Printer<double>::stream(s, indent + "  ", v.throttle);
    s << indent << "steering: ";
    Printer<double>::stream(s, indent + "  ", v.steering);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SB_MSGS_MESSAGE_CARCOMMAND_H