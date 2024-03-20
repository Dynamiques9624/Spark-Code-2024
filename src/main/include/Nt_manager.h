#pragma once

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include <iostream>
using namespace std;

class NT_Manager
{

public:
  void ntManagerInit();
  void handleSubscriberTask();

  double left_wheel_speed_percent;
  double right_wheel_speed_percent;
  double pos_value_auto;
  double ring_detected;
  double tag_detected;
  double tag_id;

  double feeder_want_down;

  double dist_front;
  double dist_rear;
  std::string pos_color;
  nt::DoublePublisher autonomous_pub;
  nt::DoublePublisher limit_switch_pub;

  nt::DoublePublisher teleop_mode_pub;
  
protected:
  

private:
  nt::NetworkTableInstance inst;
  std::shared_ptr<nt::NetworkTable> table;

  nt::DoubleSubscriber left_wheel_speed_percent_sub;
  nt::DoubleSubscriber right_wheel_speed_percent_sub;

  nt::StringSubscriber pos_color_sub;
  nt::DoubleSubscriber pos_value_sub;
  nt::DoubleSubscriber ring_detected_sub;
  nt::DoubleSubscriber tag_detected_sub;
  nt::DoubleSubscriber tag_id_sub;
  nt::DoubleSubscriber dist_front_sub;
  nt::DoubleSubscriber dist_rear_sub;
  nt::DoubleSubscriber feeder_down_sub;
};