#pragma once

#include "Config.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include <iostream>
using namespace std;

class NT_Manager{

 public:

 protected:
   double left_wheel_speed_percent;
   double right_wheel_speed_percent;
   double pos_value_auto;
   double ring_detected;
   double tag_detected;
   double tag_id;

   double dist_front;
   double dist_rear;
   std::string pos_color;

   void ntManagerInit();
   void handleSubscriberTask();

   nt::DoublePublisher autonomous_pub;
   

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
    

};