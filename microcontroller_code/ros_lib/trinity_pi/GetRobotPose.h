#ifndef _ROS_SERVICE_GetRobotPose_h
#define _ROS_SERVICE_GetRobotPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace trinity_pi
{

static const char GETROBOTPOSE[] = "trinity_pi/GetRobotPose";

  class GetRobotPoseRequest : public ros::Msg
  {
    public:

    GetRobotPoseRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETROBOTPOSE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRobotPoseResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;

    GetRobotPoseResponse():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETROBOTPOSE; };
    const char * getMD5(){ return "3f8930d968a3e84d471dff917bb1cdae"; };

  };

  class GetRobotPose {
    public:
    typedef GetRobotPoseRequest Request;
    typedef GetRobotPoseResponse Response;
  };

}
#endif
