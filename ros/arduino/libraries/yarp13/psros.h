/**
 *
 * PseudoROS Library
 *
 * V 1.02
 * 25.01.2023
 * LP 19.04.2023
 *
 */

#ifndef _PSEUDO_ROS_H_

#define _PSEUDO_ROS_H_

#include "ros/msg.h"
#include <ros/time.h>

namespace pseudoros
{

class NodeHandleBase_
{
public:
  virtual int publish(int id, const ros::Msg* msg) = 0;
  virtual int publish(const char * topic, const ros::Msg * msg) = 0;
  virtual int spinOnce() = 0;
  virtual bool connected() = 0;
};

class Subscriber_
{
public:
  virtual void callback(unsigned char *data) = 0;
  // id_ is set by NodeHandle when we advertise
  int id_;
  const char * topic_;
};

template<typename MsgT, typename ObjT = void>
class Subscriber: public Subscriber_
{
public:
  //typedef void(ObjT::*CallbackT)(const MsgT&);
  typedef void(*CallbackT)(const MsgT&);
  MsgT msg;

  Subscriber(const char *topic_name, CallbackT cb): cb_(cb)
  { topic_ = topic_name; };

  virtual void callback(unsigned char* data) override
  {
    int n = msg.deserialize(data);
    //(obj_->*cb_)(msg);
    (cb_)(msg);
  }

private:
  CallbackT cb_;
  ObjT* obj_;
};


class Publisher
{
public:
  Publisher(const char * topic_name, ros::Msg * msg):
    topic_(topic_name),
    msg_(msg)
    {};

  int publish(const ros::Msg * msg)
  {
    return nh_->publish(topic_, msg); // original version: (id_, msg)
  }

  const char * topic_;
  ros::Msg *msg_;
  // id_ and no_ are set by NodeHandle when we advertise
  int id_;
  NodeHandleBase_* nh_;
};

class Rate
{
public:
  Rate(int) {};
};


#define MSG_HEADER "###"
const char *INIT_STR = "ABCD";

template<class Hardware,
         int MAX_SUBSCRIBERS = 25,
         int MAX_PUBLISHERS = 25,
         int INPUT_SIZE = 512,
         int OUTPUT_SIZE = 512>
class NodeHandle_: public pseudoros::NodeHandleBase_
{
  public:  
    void initNode(SERIAL_CLASS* sc = &Serial, long bd = 57600)
    {
      iostream = sc;    
      baud_ = bd;
      iostream->begin(baud_);
    }

    Subscriber_ * subscribers[MAX_SUBSCRIBERS] {nullptr};
    Publisher * publishers[MAX_PUBLISHERS] = {nullptr};

    bool subscribe(pseudoros::Subscriber_ &s)
    {
      for (int i = 0; i < MAX_SUBSCRIBERS; i++)
        if(subscribers[i] == 0) // empty slot
        {
          subscribers[i] = &s;
          s.id_ = i + 100;
          return true;
        }
      return false;
    }

    pseudoros::Subscriber_ *_find_subs(char *name)
    {
      for (int i = 0; i < MAX_SUBSCRIBERS; i++)
        if(subscribers[i])
        {
          if(strcmp(subscribers[i]->topic_, name)==0)
            return subscribers[i];
        }
      return NULL;
    }
    /* Register a new publisher */
    bool advertise(pseudoros::Publisher & p)
    {
      for (int i = 0; i < MAX_PUBLISHERS; i++)
        if (publishers[i] == 0) // empty slot
        {
          publishers[i] = &p;
          p.id_ = i + 100 + MAX_SUBSCRIBERS;
          p.nh_ = this;
          return true;
        }
      return false;
    }

    virtual int publish(int id, const ros::Msg* msg) { return 1; };

    virtual int publish(const char * topic, const ros::Msg * msg)
    {
      unsigned char n = msg->serialize(outputbuffer);

      _write_block(MSG_HEADER, strlen(MSG_HEADER));
      _write_str2(topic);
      _write_byte(n);
      _write_block(outputbuffer, n);
      return n;
    }

    virtual bool connected(void) { return 1; }
    virtual int spinOnce(void) { return _eval_msg(); }

    ros::Time now()
    {
      uint32_t ms = millis(); //hardware_.time();
      ros::Time current_time;
      current_time.sec = ms / 1000;
      current_time.nsec = (ms % 1000) * 1000000UL;
      return current_time;
    }
  
    /*
     * Этих методов нет в ROS
     */
    void InitConnection(void) { _write_str(INIT_STR);  }
    void WaitForConnection(void) { _wait_str(INIT_STR);  }

protected:
    SERIAL_CLASS* iostream;
    long baud_;

    unsigned char inputbuffer[INPUT_SIZE];
    unsigned char outputbuffer[OUTPUT_SIZE];

    int _read_byte(void)
    {
      while(!iostream->available());
      return iostream->read();
    }
    int _read_block(unsigned char *buff, int n)
    {
      /** sic
      for(byte i=0;i<n;i++)
      {
        while(!iostream->available());
        buff[i] = (unsigned char)iostream->read();
      }
      **/
      n = iostream->readBytes(buff, n);
      return n;
    }

    int _write_byte(unsigned char &n) { _write_block(&n,1); return 1; }
    int _write_block(unsigned char *buff, int n) { iostream->write(buff, n); return 1; }
    int _write_str(unsigned char *str)
    {
      unsigned char n = strlen(str);
      _write_block(str,n);
      return 1;
    }
    int _write_str2(unsigned char *str)
    {
      unsigned char n = strlen(str);
      _write_block(&n,1);
      _write_block(str,n);
      return 1;
    }
    // Чтение данных до тех пор, пока не будет найдена строка str
    // Аналогичная функция есть в Arduino: find()
    void _wait_str(const char *str)
    {
      byte cp = 0;
      while(1)
        if(iostream->available())
        {
          int c = iostream->read();
          if(c==str[cp]) cp++; else cp = 0;
          if(cp>=strlen(str)) return;
        }
    }
    int _msg_ready(char *msgname)
    {
      if(!iostream->available()) return 0;
      // читаем заголовок
      _wait_str(MSG_HEADER);
      // читаем имя
      unsigned char slen;
      _read_block(&slen, 1);
      _read_block(msgname, slen);
      msgname[slen] = 0;
      return 1;
    }

    int _eval_msg(void)
    {
      char s[20];
      if(_msg_ready(s))
      {
        pseudoros::Subscriber_ *subs = _find_subs(s);
        if(subs)
        {
          int blen = _read_byte();
          _read_block(inputbuffer, blen);
          subs->callback(inputbuffer);
          return 1;
        }
      }
      return 0;
    }
};

};

namespace pseudoros
{

static const char _swrk[20];
char *i2str(const char *fmt, int r)
{
  sprintf(_swrk, fmt, r);
  return _swrk;
}

char *f2str(const char *prefix, float r, int len=5, int dec=3)
{
  char s[20];  
  dtostrf(r, len, dec, s);  
  strcpy(_swrk, prefix);
  strcat(_swrk, s);
  return _swrk;
}


// Задержка, но с выполнением функции
void usr_delay_ms(long ms, int(*usrf)(void))
{
  long ctt = millis();
  while((millis()-ctt) < ms)
    if(usrf) usrf();
}
};

#endif
