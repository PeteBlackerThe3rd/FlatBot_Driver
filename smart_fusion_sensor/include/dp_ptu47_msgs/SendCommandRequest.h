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
 * Auto-generated by genmsg_cpp from file /home/brian/hydro_catkin_ws/smp_new/src/dp_ptu47_msgs/srv/SendCommand.srv
 *
 */


#ifndef DP_PTU47_MSGS_MESSAGE_SENDCOMMANDREQUEST_H
#define DP_PTU47_MSGS_MESSAGE_SENDCOMMANDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dp_ptu47_msgs
{
template <class ContainerAllocator>
struct SendCommandRequest_
{
  typedef SendCommandRequest_<ContainerAllocator> Type;

  SendCommandRequest_()
    : pan_angle(0.0)
    , tilt_angle(0.0)
    , wait_finished(false)  {
    }
  SendCommandRequest_(const ContainerAllocator& _alloc)
    : pan_angle(0.0)
    , tilt_angle(0.0)
    , wait_finished(false)  {
    }



   typedef float _pan_angle_type;
  _pan_angle_type pan_angle;

   typedef float _tilt_angle_type;
  _tilt_angle_type tilt_angle;

   typedef uint8_t _wait_finished_type;
  _wait_finished_type wait_finished;




  typedef boost::shared_ptr< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct SendCommandRequest_

typedef ::dp_ptu47_msgs::SendCommandRequest_<std::allocator<void> > SendCommandRequest;

typedef boost::shared_ptr< ::dp_ptu47_msgs::SendCommandRequest > SendCommandRequestPtr;
typedef boost::shared_ptr< ::dp_ptu47_msgs::SendCommandRequest const> SendCommandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dp_ptu47_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'dp_ptu47_msgs': ['/home/brian/hydro_catkin_ws/smp_new/src/dp_ptu47_msgs/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab01edbbd34c096e8435c610b2318221";
  }

  static const char* value(const ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab01edbbd34c096eULL;
  static const uint64_t static_value2 = 0x8435c610b2318221ULL;
};

template<class ContainerAllocator>
struct DataType< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dp_ptu47_msgs/SendCommandRequest";
  }

  static const char* value(const ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pan_angle\n\
float32 tilt_angle\n\
bool wait_finished\n\
";
  }

  static const char* value(const ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pan_angle);
      stream.next(m.tilt_angle);
      stream.next(m.wait_finished);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct SendCommandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dp_ptu47_msgs::SendCommandRequest_<ContainerAllocator>& v)
  {
    s << indent << "pan_angle: ";
    Printer<float>::stream(s, indent + "  ", v.pan_angle);
    s << indent << "tilt_angle: ";
    Printer<float>::stream(s, indent + "  ", v.tilt_angle);
    s << indent << "wait_finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wait_finished);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DP_PTU47_MSGS_MESSAGE_SENDCOMMANDREQUEST_H