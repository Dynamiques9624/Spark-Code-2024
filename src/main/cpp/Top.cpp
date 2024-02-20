#include "Top.h"

void Top:: handleTopInit(){

    m_dutyCycleEncoder_feeder.SetDistancePerRotation(360);
    m_dutyCycleEncoder_lancer.SetDistancePerRotation(360);

    motor_feeder.RestoreFactoryDefaults();
    motor_lancer_an_left.RestoreFactoryDefaults();
    motor_lancer_an_right.RestoreFactoryDefaults();

    motor_bascul_left.RestoreFactoryDefaults();
    motor_bascul_right.RestoreFactoryDefaults();

    bascul_value = -30;

}

void Top::handleEncoderValue(){
    connect_encoder_feeder = m_dutyCycleEncoder_feeder.IsConnected();
    connect_encoder_lancer = m_dutyCycleEncoder_lancer.IsConnected();

    angle_encoder_feeder = m_dutyCycleEncoder_feeder.GetDistance();
    angle_encoder_lancer = m_dutyCycleEncoder_lancer.GetDistance();

    frc::SmartDashboard::PutBoolean("encoder feeder", connect_encoder_feeder);
    frc::SmartDashboard::PutBoolean("encoder lancer", connect_encoder_lancer);
    
}


void Top::PositionFeeder(){ 
    RB = controller.GetRightBumper();
    LB = controller.GetLeftBumper();
    RT = controller.GetRightTriggerAxis();
    if(LB){
        if(angle_encoder_feeder < ENCODER_FEEDER_TAKE_VALUE){
             motor_feeder.Set(-MOTOR_FEEDER_SPEED);
        }   
    }
    if(RB){
        if(angle_encoder_feeder > ENCODER_FEEDER_LANCER_VALUE){
            motor_feeder.Set(MOTOR_FEEDER_SPEED);  
        }
    }

    if(angle_encoder_feeder < ENCODER_FEEDER_LANCER_VALUE){
        motor_feeder.Set(0.00001);
        feeder_possition_lancer_state = 1;
    }

    if(angle_encoder_feeder > ENCODER_FEEDER_TAKE_VALUE){
        motor_feeder.Set(0.00001);
           feeder_possition_lancer_state = 0;
    }
}
void Top::PositionBasculTele(){
    pov = controller.GetPOV();

    if (pov == 0 && bascul_value < MAX_VALUE_BASCUL){
        bascul_value += 0.5;
    }
    if(pov == 180 && bascul_value > MIN_VALUE_BASCUL){
        bascul_value -=0.5;
    }

    bascul_value_aprox = abs(bascul_value)-abs(angle_encoder_feeder);
    frc::SmartDashboard::PutNumber("basculencoder", angle_encoder_lancer);
    frc::SmartDashboard::PutNumber("basculValue", bascul_value);
    std::cout<<"distance" <<angle_encoder_lancer <<"\n";
    //std::cout<<"anglevalue" <<bascul_value <<"\n";  
    
    

    if(bascul_value > angle_encoder_lancer && bascul_value_aprox < -10){
        //bascule monte (pointe vers haut)
        motor_bascul_left.Set(-0.2);
        motor_bascul_right.Set(0.2);
        
    }
    if(bascul_value < angle_encoder_lancer && bascul_value_aprox > 10){
        //bascule descend (point vers bas)
        motor_bascul_left.Set(0.2);
        motor_bascul_right.Set(-0.2);
        
    }
    if ( bascul_value_aprox <= 1 && bascul_value_aprox >= 10){
            motor_bascul_left.Set(0.00001);
            motor_bascul_right.Set(-0.00001);
    }
    if ( bascul_value_aprox >= -10 && bascul_value_aprox < 0 ){
            motor_bascul_left.Set(0.00001);
            motor_bascul_right.Set(-0.00001);
    }
}

void Top::handlelancerSpeed(){

    motor_lancer_an_right.Set(-0.6);
    motor_lancer_an_left.Set(0.6);
}

void Top::intakeAnneau(){

    RT = controller.GetRightTriggerAxis();
    

    if (RT == 1){
        m_motor_intake.Set(-0.5);
    }
    if (anneau_limit_Switch.Get() && RT != 1){
        m_motor_intake.Set(0);
    }
    if(feeder_possition_lancer_state == 0 && anneau_limit_Switch.Get() == false ){
        m_motor_intake.Set(0.2);
        
    }
}

void Top::handleTopTaskTeleop(){
    handleEncoderValue();
    //handlelancerSpeed();
    //PositionFeeder();
    PositionBasculTele();
    //intakeAnneau();

}