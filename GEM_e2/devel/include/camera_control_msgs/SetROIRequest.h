// Generated by gencpp from file camera_control_msgs/SetROIRequest.msg
// DO NOT EDIT!


#ifndef CAMERA_CONTROL_MSGS_MESSAGE_SETROIREQUEST_H
#define CAMERA_CONTROL_MSGS_MESSAGE_SETROIREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/RegionOfInterest.h>

namespace camera_control_msgs
{
template <class ContainerAllocator>
struct SetROIRequest_
{
  typedef SetROIRequest_<ContainerAllocator> Type;

  SetROIRequest_()
    : target_roi()  {
    }
  SetROIRequest_(const ContainerAllocator& _alloc)
    : target_roi(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  _target_roi_type;
  _target_roi_type target_roi;





  typedef boost::shared_ptr< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetROIRequest_

typedef ::camera_control_msgs::SetROIRequest_<std::allocator<void> > SetROIRequest;

typedef boost::shared_ptr< ::camera_control_msgs::SetROIRequest > SetROIRequestPtr;
typedef boost::shared_ptr< ::camera_control_msgs::SetROIRequest const> SetROIRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::camera_control_msgs::SetROIRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::camera_control_msgs::SetROIRequest_<ContainerAllocator1> & lhs, const ::camera_control_msgs::SetROIRequest_<ContainerAllocator2> & rhs)
{
  return lhs.target_roi == rhs.target_roi;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::camera_control_msgs::SetROIRequest_<ContainerAllocator1> & lhs, const ::camera_control_msgs::SetROIRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace camera_control_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cf55ea464b4556def55bfcda0d3eab55";
  }

  static const char* value(const ::camera_control_msgs::SetROIRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcf55ea464b4556deULL;
  static const uint64_t static_value2 = 0xf55bfcda0d3eab55ULL;
};

template<class ContainerAllocator>
struct DataType< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "camera_control_msgs/SetROIRequest";
  }

  static const char* value(const ::camera_control_msgs::SetROIRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Select a region of interest to get a cropped image.\n"
"# The region is defined by four parameters\n"
"# roi.width: with of the region\n"
"# roi.height: height of the region\n"
"# roi.x_offset at which pixel a long the x axis (horizontal) does the\n"
"# cropped region start\n"
"# roi.y_offset at which pixel a long the y axis (vertical) does the\n"
"# cropped region start\n"
"# The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]\n"
"# Notice that x_offset cannot be larger than img.width - roi.width\n"
"# The same for y_offset, not larger than img.height - roi.height\n"
"sensor_msgs/RegionOfInterest target_roi\n"
"\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/RegionOfInterest\n"
"# This message is used to specify a region of interest within an image.\n"
"#\n"
"# When used to specify the ROI setting of the camera when the image was\n"
"# taken, the height and width fields should either match the height and\n"
"# width fields for the associated image; or height = width = 0\n"
"# indicates that the full resolution image was captured.\n"
"\n"
"uint32 x_offset  # Leftmost pixel of the ROI\n"
"                 # (0 if the ROI includes the left edge of the image)\n"
"uint32 y_offset  # Topmost pixel of the ROI\n"
"                 # (0 if the ROI includes the top edge of the image)\n"
"uint32 height    # Height of ROI\n"
"uint32 width     # Width of ROI\n"
"\n"
"# True if a distinct rectified ROI should be calculated from the \"raw\"\n"
"# ROI in this message. Typically this should be False if the full image\n"
"# is captured (ROI not used), and True if a subwindow is captured (ROI\n"
"# used).\n"
"bool do_rectify\n"
;
  }

  static const char* value(const ::camera_control_msgs::SetROIRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_roi);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetROIRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::camera_control_msgs::SetROIRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::camera_control_msgs::SetROIRequest_<ContainerAllocator>& v)
  {
    s << indent << "target_roi: ";
    s << std::endl;
    Printer< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> >::stream(s, indent + "  ", v.target_roi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAMERA_CONTROL_MSGS_MESSAGE_SETROIREQUEST_H