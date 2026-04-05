#ifndef _ROS_msg_senfield2_senfield_h
#define _ROS_msg_senfield2_senfield_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "msg_kvorum2/sdata.h"
#include "msg_crtcoord/crtcoord.h"

namespace msg_senfield2
{

  class senfield : public ros::Msg
  {
    public:
      typedef msg_kvorum2::sdata _arddata_type;
      _arddata_type arddata;
      typedef msg_kvorum2::sdata _ardreg_type;
      _ardreg_type ardreg;
      typedef msg_kvorum2::sdata _ardi2c_type;
      _ardi2c_type ardi2c;
      typedef msg_kvorum2::sdata _ardstatus_type;
      _ardstatus_type ardstatus;
      typedef msg_kvorum2::sdata _pll_type;
      _pll_type pll;
      typedef msg_kvorum2::sdata _qrc_type;
      _qrc_type qrc;
      typedef msg_kvorum2::sdata _face_type;
      _face_type face;
      typedef msg_kvorum2::sdata _spc_type;
      _spc_type spc;
      typedef msg_kvorum2::sdata _usrcmd_type;
      _usrcmd_type usrcmd;
      typedef msg_kvorum2::sdata _kslot_type;
      _kslot_type kslot;
      typedef msg_crtcoord::crtcoord _robotcoord_type;
      _robotcoord_type robotcoord;

    senfield():
      arddata(),
      ardreg(),
      ardi2c(),
      ardstatus(),
      pll(),
      qrc(),
      face(),
      spc(),
      usrcmd(),
      kslot(),
      robotcoord()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->arddata.serialize(outbuffer + offset);
      offset += this->ardreg.serialize(outbuffer + offset);
      offset += this->ardi2c.serialize(outbuffer + offset);
      offset += this->ardstatus.serialize(outbuffer + offset);
      offset += this->pll.serialize(outbuffer + offset);
      offset += this->qrc.serialize(outbuffer + offset);
      offset += this->face.serialize(outbuffer + offset);
      offset += this->spc.serialize(outbuffer + offset);
      offset += this->usrcmd.serialize(outbuffer + offset);
      offset += this->kslot.serialize(outbuffer + offset);
      offset += this->robotcoord.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->arddata.deserialize(inbuffer + offset);
      offset += this->ardreg.deserialize(inbuffer + offset);
      offset += this->ardi2c.deserialize(inbuffer + offset);
      offset += this->ardstatus.deserialize(inbuffer + offset);
      offset += this->pll.deserialize(inbuffer + offset);
      offset += this->qrc.deserialize(inbuffer + offset);
      offset += this->face.deserialize(inbuffer + offset);
      offset += this->spc.deserialize(inbuffer + offset);
      offset += this->usrcmd.deserialize(inbuffer + offset);
      offset += this->kslot.deserialize(inbuffer + offset);
      offset += this->robotcoord.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "msg_senfield2/senfield"; };
    virtual const char * getMD5() override { return "df9ac5e175f32770121dc4b649208f55"; };

  };

}
#endif
