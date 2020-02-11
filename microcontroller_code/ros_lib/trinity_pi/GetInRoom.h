#ifndef _ROS_SERVICE_GetInRoom_h
#define _ROS_SERVICE_GetInRoom_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Bool.h"

namespace trinity_pi
{

static const char GETINROOM[] = "trinity_pi/GetInRoom";

  class GetInRoomRequest : public ros::Msg
  {
    public:

    GetInRoomRequest()
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

    const char * getType(){ return GETINROOM; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetInRoomResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Bool _inRoom_type;
      _inRoom_type inRoom;

    GetInRoomResponse():
      inRoom()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->inRoom.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->inRoom.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETINROOM; };
    const char * getMD5(){ return "e3d48fee9728aec81883ede64d1f1dea"; };

  };

  class GetInRoom {
    public:
    typedef GetInRoomRequest Request;
    typedef GetInRoomResponse Response;
  };

}
#endif
