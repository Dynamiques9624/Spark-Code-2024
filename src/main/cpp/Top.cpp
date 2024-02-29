#include "Top.h"

void Top:: handleTopInit(){
    feederInit();

    m_dutyCycleEncoder_lancer.SetDistancePerRotation(360);

    motor_lancer_an_left.RestoreFactoryDefaults();
    motor_lancer_an_right.RestoreFactoryDefaults();

    motor_bascul_left.RestoreFactoryDefaults();
    motor_bascul_right.RestoreFactoryDefaults();

    bascul_value = -20;
    set_predefine_loaded = false;

    timer_started_propul = false;
    timer_started_fire = false;
    
    bascul_target_position = false;

    y_button_handler = false;
  
}

void Top::handleEncoderValue(){
    
    connect_encoder_lancer = m_dutyCycleEncoder_lancer.IsConnected();
    angle_encoder_lancer = m_dutyCycleEncoder_lancer.GetDistance();
    frc::SmartDashboard::PutNumber("encoLancerVal", angle_encoder_lancer);
    frc::SmartDashboard::PutBoolean("encoder lancer", connect_encoder_lancer);
    
}

void Top::handleMotorTemp(){
    
    temp_m_lancer_left = motor_lancer_an_left.GetMotorTemperature();
    temp_m_lancer_right = motor_lancer_an_right.GetMotorTemperature();
    temp_m_bascul_left =  motor_bascul_left.GetMotorTemperature();
    temp_m_bascul_right =  motor_bascul_right.GetMotorTemperature();

    frc::SmartDashboard::PutNumber("tempMlancerLeft", temp_m_lancer_left);
    frc::SmartDashboard::PutNumber("tempMlancerRight", temp_m_lancer_right);
    frc::SmartDashboard::PutNumber("tempMbasculLeft", temp_m_bascul_left);
    frc::SmartDashboard::PutNumber("tempMbasculRight", temp_m_bascul_right);

}



void Top::PositionBascul(){
    pov = controller.GetPOV();
    LT = controller.GetLeftTriggerAxis();

    if (feeder_state != Loaded && feeder_state != Fire){
        set_predefine_loaded = true;
    }else if (set_predefine_loaded) {
        bascul_value = -25;
        set_predefine_loaded = false;
    }
    
    if (LT == 1 && bascul_value < MAX_VALUE_BASCUL){
        bascul_value += 0.5;
    }
    if(pov == 0 && bascul_value > MIN_VALUE_BASCUL){
        bascul_value -=0.5;
    }
    basculHandler();
}

void Top::basculHandler(){

    bascul_value_aprox = abs(bascul_value)-abs(angle_encoder_lancer);

    frc::SmartDashboard::PutNumber("basculencoder", angle_encoder_lancer);
    frc::SmartDashboard::PutNumber("basculValue", bascul_value);
    frc::SmartDashboard::PutNumber("valueArox", bascul_value_aprox);  
    
    if(bascul_value > angle_encoder_lancer && bascul_value_aprox < -2){
        //bascule monte (pointe vers haut)
        motor_bascul_left.Set(0);
        motor_bascul_right.Set(0);
        
    }
    if(bascul_value < angle_encoder_lancer && bascul_value_aprox > 2){
        //bascule descend (point vers bas)
        motor_bascul_left.Set(0.2);
        motor_bascul_right.Set(-0.2);
        
    }

    bascul_target_position = false;

    if (bascul_value_aprox <= 2 && bascul_value_aprox >= 0){
        motor_bascul_left.Set(0.00001);
        motor_bascul_right.Set(-0.00001);
        bascul_target_position = true;
        
    }
    if (bascul_value_aprox >= -2 && bascul_value_aprox < 0 ){
        motor_bascul_left.Set(0.00001);
        motor_bascul_right.Set(-0.00001);
        bascul_target_position = true;
    }
}

void Top::handlelancerSpeed(){
    yButtonHandler();
    

    if(feeder_state == Loaded || feeder_state == Fire){
        if (y_button_handler){
            motor_lancer_an_right.Set(-0.8);
            motor_lancer_an_left.Set(0.8);
        }
    }
    else{
        motor_lancer_an_right.Set(0);
        motor_lancer_an_left.Set(0);
    }
 
}

void Top::yButtonHandler(){
    y_button = controller.GetYButton();

    if (y_button && !y_button_handler){
        y_button_handler = true;
    }else if (y_button_handler){
        y_button_handler = false;
    }
}

void Top::handleTopTaskTeleop(){
    feederHandler();
    handleEncoderValue();
    handleMotorTemp();
    handlelancerSpeed();
    PositionBascul();
    ampHandler();

}

void Top::handleTopTaskAuto(){
    handleEncoderValue();
    lanceurAuto();
    handlelancerSpeed();
    feederHandler();
    PositionBascul();    
}

void Top::lanceurAuto(){

    if (feeder_state == Loaded){

        motor_lancer_an_right.Set(-0.6);
        motor_lancer_an_left.Set(0.6);
        bascul_value = -25;
        startTimerPropul();
        time_t delay_propul = time(NULL) - start_propul;

        if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
            feederFireAuto();
            startTimerFire();
            time_t delay_fire = time(NULL) - start_fire;
            if (delay_fire >= WAIT_TIME_SHOOTHING){
                Feeder::setState(FeederState::Idle);
                bascul_value = -20;
            }
        } 
    }    
        

}

void Top::startTimerPropul(){
    
    if (!timer_started_propul){
        time(&start_propul);
        timer_started_propul = true;
    }
}

void Top::startTimerFire(){
    if (!timer_started_fire){
        time(&start_fire);
        timer_started_fire = true;
    }
}

//-------------------------------------------------------------------

void Top::ampHandler(){
    switch (amp_state)
    {
    case Idle:
        basculIdle();
        break;
    case BasculUp:
        basculUp();
        break;
    case BasculGoingUp:
        basculGoingUp();
        break;
    case FeederUp:
        feederUp();
        break;
    case Loaded:
        feederLoaded();
        break;
    case Fire:
        feederFire();
        break;
    case FeederDown:
        feederDown();
        break;
    case BasculDown:
        basculDown();
        break;
    case BasculGoingDown:
        basculGoingDown();
        break;

        
    
    default:
        break;
    }
}

void Top::basculUp(){
    bascul_value = BASCUL_VALUE_AMP;
    basculHandler();
    setState(AmpState::BasculGoingUp);

}

void Top::basculIdle(){
    LB = controller.GetLeftBumper();

    if(LB){
        setState(AmpState::BasculUp);

    }
}

void Top::basculGoingUp(){
    if (bascul_target_position){
        setState(AmpState::FeederUp);
    }
}

void Top::feederUp(){
    Feeder::feederEncoderReader();
    if(angle_encoder_feeder < AMP_POSITION_FEEDER_DOWN){ 
        if(feeder_speed != FEEDER_DOWN_SPEED){
            feeder_speed = FEEDER_DOWN_SPEED;
            setMotorFeeder(feeder_speed);

        }
        
    }else{
        feeder_speed = 0;
        setMotorFeeder(feeder_speed);
        setState(AmpState::Loaded);

    }
}

void Top::feederLoaded(){
    RT = controller.GetRightTriggerAxis();

    if(RT != 1){
        return;
    }

    intake_speed = INTAKE_PUSH_SPEED;
    setMotorIntake(intake_speed);
    setState(AmpState::Fire);
    
}

void Top::feederFire(){
    RT = controller.GetRightTriggerAxis();
    
    if(RT != 1){
        intake_speed = 0;
        setMotorIntake(intake_speed);
        setState(AmpState::FeederDown);
        
    }
}

void Top::feederDown(){
    Feeder::feederEncoderReader();
    if(angle_encoder_feeder > ENCODER_FEEDER_LANCER_VALUE){ 
        if(feeder_speed != FEEDER_UP_MAX_SPEED){
            feeder_speed = FEEDER_UP_MAX_SPEED;
            setMotorFeeder(feeder_speed);

        }
        
    }else{
        feeder_speed = 0;
        setMotorFeeder(feeder_speed);
        setState(AmpState::BasculDown);

    }
}

void Top::basculDown(){
    bascul_value = MAX_VALUE_BASCUL;
    basculHandler();
    setState(AmpState::BasculGoingDown);
}

void Top::basculGoingDown(){
    if (bascul_target_position){
        setState(AmpState::Idle);
    
    }
}

void Top::setState(AmpState state){
    std::cout <<"amp_state " << state << "\n";
    frc::SmartDashboard::PutNumber("AmpState", state);
    amp_state = state; 
}