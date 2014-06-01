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
 * Auto-generated by genmsg_cpp from file /home/jannicke/catkin_ws/src/sb_msgs/msg/LidarNav.msg
 *
 */


#ifndef SB_MSGS_MESSAGE_LIDARNAV_H
#define SB_MSGS_MESSAGE_LIDARNAV_H


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
struct LidarNav_
{
  typedef LidarNav_<ContainerAllocator> Type;

  LidarNav_()
    : direction(0.0)
    , distance(0.0)
    , confidence(0)  {
    }
  LidarNav_(const ContainerAllocator& _alloc)
    : direction(0.0)
    , distance(0.0)
    , confidence(0)  {
    }



   typedef float _direction_type;
  _direction_type direction;

   typedef float _distance_type;
  _distance_type distance;

   typedef int32_t _confidence_type;
  _confidence_type confidence;




  typedef boost::shared_ptr< ::sb_msgs::LidarNav_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sb_msgs::LidarNav_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct LidarNav_

typedef ::sb_msgs::LidarNav_<std::allocator<void> > LidarNav;

typedef boost::shared_ptr< ::sb_msgs::LidarNav > LidarNavPtr;
typedef boost::shared_ptr< ::sb_msgs::LidarNav const> LidarNavConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sb_msgs::LidarNav_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sb_msgs::LidarNav_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::sb_msgs::LidarNav_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sb_msgs::LidarNav_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sb_msgs::LidarNav_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sb_msgs::LidarNav_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sb_msgs::LidarNav_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sb_msgs::LidarNav_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sb_msgs::LidarNav_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f176eab4d8b8cab198d7795b194bf566";
  }

  static const char* value(const ::sb_msgs::LidarNav_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf176eab4d8b8cab1ULL;
  static const uint64_t static_value2 = 0x98d7795b194bf566ULL;
};

template<class ContainerAllocator>
struct DataType< ::sb_msgs::LidarNav_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sb_msgs/LidarNav";
  }

  static const char* value(const ::sb_msgs::LidarNav_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sb_msgs::LidarNav_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Message from LidarNav node to commander\n\
\n\
float32 direction\n\
float32 distance\n\
int32 confidence #a percentage\n\
";
  }

  static const char* value(const ::sb_msgs::LidarNav_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sb_msgs::LidarNav_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.direction);
      stream.next(m.distance);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct LidarNav_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sb_msgs::LidarNav_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sb_msgs::LidarNav_<ContainerAllocator>& v)
  {
    s << indent << "direction: ";
    Printer<float>::stream(s, indent + "  ", v.direction);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
    s << indent << "confidence: ";
    Printer<int32_t>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SB_MSGS_MESSAGE_LIDARNAV_H