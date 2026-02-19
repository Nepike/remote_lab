/*
 * Example has taken from
 * https://github.com/osrf/gazebo/blob/gazebo11/examples/stand_alone/listener/listener.cc
*/

#include <mutex>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

ros::Publisher contact_pub;
std::string bumper_gazebo_link;
bool collision_found = false;
bool setup_output = false;
std::mutex data_mutex;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb_contacts(ConstContactsPtr &_msg)
{
  std_msgs::Bool result_msg;
  //ROS_ERROR("proc");
  for( size_t i = 0 ; i < _msg->contact_size() ; i++){
      // if one of contacts starts from bumper link name, 
      // collision found
      if (setup_output)
          ROS_ERROR("%s - %s", _msg->contact(i).collision1().c_str(), _msg->contact(i).collision2().c_str());
      const std::string& link1 = _msg->contact(i).collision1();
      const std::string& link2 = _msg->contact(i).collision2();
      if (link1 == bumper_gazebo_link || link2 == bumper_gazebo_link) {
          data_mutex.lock();
          collision_found = true;
          data_mutex.unlock();
          break;
      }
  }
  contact_pub.publish(result_msg);
}

void cb_timer(const ros::TimerEvent&) {
    std_msgs::Bool result_msg;
    data_mutex.lock();
    result_msg.data = collision_found;
    collision_found = false;
    data_mutex.unlock();
    contact_pub.publish(result_msg);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{  

  ros::init(_argc, _argv, "bumper_handler");      
  ros::NodeHandle nh_p("~");
  
  // bumper name
  if (!nh_p.hasParam("bumper_gazebo_link"))
      bumper_gazebo_link = "test_robot::base_link::base_link_fixed_joint_lump__bumper_collision";
  else
      nh_p.getParam("bumper_gazebo_link", bumper_gazebo_link);
  // timer frequency
  float timer_freq = 10.;
  if (nh_p.hasParam("timer_freq"))
      nh_p.getParam("timer_freq", timer_freq);

  if (nh_p.hasParam("setup_output"))
      nh_p.getParam("setup_output", setup_output);
  
  contact_pub = nh_p.advertise<std_msgs::Bool>("has_contact",1);
  
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", cb_contacts);
  ros::Timer timer = nh_p.createTimer(ros::Duration(1. / timer_freq), cb_timer);
  
  
  // Busy wait loop...replace with your own code as needed.
  ros::spin();
//   while (true)
//     gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
