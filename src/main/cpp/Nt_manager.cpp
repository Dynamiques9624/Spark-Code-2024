#include "Nt_manager.h"
#include "Config.h"



void NT_Manager::ntManagerInit(){
    inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable(TOPIC);
    left_wheel_speed_percent_sub = table->GetDoubleTopic("left_wheel_speed").Subscribe(0.0);
    right_wheel_speed_percent_sub = table->GetDoubleTopic("right_wheel_speed").Subscribe(0.0);
    pos_value_sub = table->GetDoubleTopic("pos_value").Subscribe(0.0);
    pos_color_sub = table->GetStringTopic("pos_color").Subscribe("");
    ring_detected_sub = table->GetDoubleTopic("ring_detected").Subscribe(0.0);
    tag_detected_sub = table->GetDoubleTopic("tag_detected").Subscribe(0.0);
    tag_id_sub = table->GetDoubleTopic("tag_id").Subscribe(0.0);
    dist_front_sub = table->GetDoubleTopic("dist_front").Subscribe(0.0);
    dist_rear_sub = table->GetDoubleTopic("dist_rear").Subscribe(0.0);
    feeder_down_sub = table->GetDoubleTopic("feeder_down").Subscribe(0.0);

    auto autonumous = table->GetDoubleTopic("autonomous");
    autonomous_pub = autonumous.Publish();

    auto teleop = table->GetDoubleTopic("teleop_mode");
    teleop_mode_pub = teleop.Publish();
    
    auto switch_intake = table->GetDoubleTopic("limit_switch");
    limit_switch_pub = switch_intake.Publish();

}

// ----------------------------------------------------------------------------
//

void NT_Manager::handleSubscriberTask(){
    this->left_wheel_speed_percent = left_wheel_speed_percent_sub.Get();
    //std::cout <<"leftSpeedAuto " << left_wheel_speed_percent << "\n";

    this->right_wheel_speed_percent = right_wheel_speed_percent_sub.Get();
    //std::cout <<"rightSpeedAuto " << right_wheel_speed_percent << "\n";

    this->feeder_want_down = feeder_down_sub.Get();
    //std::cout <<"feederWantDown " << feeder_want_down << "\n";

    this->pos_value_auto = pos_value_sub.Get();
    //std::cout <<"posValueAuto " << pos_value_auto << "\n";

    this->pos_color = pos_color_sub.Get();
    //std::cout <<"posColorAuto " << pos_color << "\n";


    //-1 = pas de ring 1 = ring
    this->ring_detected = ring_detected_sub.Get();
    //std::cout <<"ringDetectionValue " << pos_color << "\n";

    //-1 = pas de tag 1 = tag
    this->tag_detected = tag_detected_sub.Get();
    //std::cout <<"tagDetected " << tag_detected << "\n";

    this->tag_id = tag_id_sub.Get();
    //std::cout <<"tagId " << tag_id << "\n";

    this->dist_rear = dist_rear_sub.Get();
    //std::cout <<"distRear " << dist_rear << "\n";

    this->dist_front = dist_front_sub.Get();
    //std::cout <<"distFront " << dist_front << "\n";
}