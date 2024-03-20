#include "Top.h"

void Top:: handleTopInit(){
    feederInit();

    m_dutyCycleEncoder_lancer.SetDistancePerRotation(360);

    motor_lancer_an_left.RestoreFactoryDefaults();
    motor_lancer_an_right.RestoreFactoryDefaults();

    motor_bascul_left.RestoreFactoryDefaults();
    motor_bascul_right.RestoreFactoryDefaults();

    bascul_value = -81.5;
    set_predefine_loaded = false;

    timer_started_propul = false;
    timer_started_fire = false;
    timer_started_relay = false;

    bascul_target_position = false;

    relay_activited = false;
    
    y_button_handler = false;
  
}

// ----------------------------------------------------------------------------
//

void Top::handleTopAutoInit(){

    
}

// ----------------------------------------------------------------------------
//

void Top::handleEncoderValue(){
    
    connect_encoder_lancer = m_dutyCycleEncoder_lancer.IsConnected();
    angle_encoder_lancer = m_dutyCycleEncoder_lancer.GetDistance();
    frc::SmartDashboard::PutNumber("encoLancerVal", angle_encoder_lancer);
    frc::SmartDashboard::PutBoolean("encoder lancer", connect_encoder_lancer);
    
}
// ----------------------------------------------------------------------------
//

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
// ----------------------------------------------------------------------------
//



void Top::PositionBascul(){
    pov = m_controller.GetPOV();
    LT = m_controller.GetLeftTriggerAxis();

    if (feeder_state != FeederState::Loaded && feeder_state != FeederState::Fire){
        set_predefine_loaded = true;
    }else if (set_predefine_loaded) {
        bascul_value = -86.5;
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
// ----------------------------------------------------------------------------
//

void Top::basculHandler(){

    bascul_value_aprox = abs(bascul_value)-abs(angle_encoder_lancer);

    frc::SmartDashboard::PutNumber("basculencoder", angle_encoder_lancer);
    frc::SmartDashboard::PutNumber("basculValue", bascul_value);
    frc::SmartDashboard::PutNumber("valueArox", bascul_value_aprox);  
    
    if(bascul_value > angle_encoder_lancer && bascul_value_aprox < -2){
        //bascule monte (pointe vers haut)
        motor_bascul_left.Set(-bascul_down_speed); //-0.08
        motor_bascul_right.Set(bascul_down_speed); //0.08
        
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
// ----------------------------------------------------------------------------
//

void Top::handlelancerSpeed(){
    yButtonHandler();
    

    if(feeder_state == FeederState::GoUp || feeder_state == FeederState::Loaded || feeder_state == FeederState::Fire){
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
// ----------------------------------------------------------------------------
//

void Top::yButtonHandler(){
    y_button = m_second_controller.GetYButton();

    if (y_button && !y_button_handler){
        y_button_handler = true;
    }else if (y_button_handler){
        y_button_handler = false;
    }
}
// ----------------------------------------------------------------------------
//

void Top::lanceurAutoOneNote(){

    if (feeder_state == Loaded){

        motor_lancer_an_right.Set(-0.6);
        motor_lancer_an_left.Set(0.6);
        bascul_value = -86.5;
        startTimerPropul();
        time_t delay_propul = time(NULL) - start_propul;

        if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
            feederFireAuto();
            startTimerFire();
            time_t delay_fire = time(NULL) - start_fire;
            if (delay_fire >= WAIT_TIME_SHOOTHING){
                Feeder::setState(FeederState::Idle);
                first_note_shoot_append = true;
                bascul_value = -81.5;
            }
        } 
    } 
        

}

// ----------------------------------------------------------------------------
//

void Top::lanceurAutoFirstOfTwo(){

    if (feeder_state == Loaded){

        motor_lancer_an_right.Set(-0.6);
        motor_lancer_an_left.Set(0.6);
        bascul_value = -86.5;
        startTimerPropul();
        time_t delay_propul = time(NULL) - start_propul;

        if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
            feederFireAuto();
            startTimerFire();
            time_t delay_fire = time(NULL) - start_fire;
            if (delay_fire >= WAIT_TIME_SHOOTHING){
                Feeder::setState(FeederState::GoDown);
                first_note_shoot_append = true;
                bascul_value = -81.5;
            }
        } 
    } 
        

}
// ----------------------------------------------------------------------------
//

void Top::lanceurAutoTwoOfTwo(){

    if (feeder_state == Loaded){

        motor_lancer_an_right.Set(-0.8);
        motor_lancer_an_left.Set(0.8);
        bascul_value = SHOOT_SECOND_NOTE_AUTO;
        startTimerPropul();
        time_t delay_propul = time(NULL) - start_propul;

        if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
            feederFireAuto();
            startTimerFire();
            time_t delay_fire = time(NULL) - start_fire;
            if (delay_fire >= WAIT_TIME_SHOOTHING){
                Feeder::setState(FeederState::Idle);
                second_note_shoot_append = true;
                bascul_value = -81.5;
            }
        } 
    } 
        

}


// ----------------------------------------------------------------------------
//

void Top::startTimerPropul(){
    
    if (!timer_started_propul){
        time(&start_propul);
        timer_started_propul = true;
    }
}

// ----------------------------------------------------------------------------
//

void Top::startTimerFire(){
    if (!timer_started_fire){
        time(&start_fire);
        timer_started_fire = true;
    }
}

// ----------------------------------------------------------------------------
//

void Top::handleTopTaskTeleop(){
    feederHandler();
    handleEncoderValue();
    handleMotorTemp();
    handlelancerSpeed();
    PositionBascul();
    ampHandler();
    handleRelaySolo();

}
// ----------------------------------------------------------------------------
//

void Top::handleTopTaskAuto(){
    handleEncoderValue();
    handlelancerSpeed();
    feederHandler();
    PositionBascul();

    if (!pos_auto_midle){
        lanceurAutoOneNote(); 
    }else if (pos_auto_midle){
        if (!first_note_shoot_append){
            lanceurAutoFirstOfTwo();
        }else if (!second_note_shoot_append){
            lanceurAutoTwoOfTwo();
        }
        
    }
    
    
}

// ----------------------------------------------------------------------------
//

void Top::handleRelaySolo(){

    pov_second = m_second_controller.GetPOV();
    

    if (pov_second == 0){
        if (!relay_activited){
            std::cout <<"relay " << "on" << "\n";
            relay_solo.Set(1);
            relay_activited = true;
            frc::SmartDashboard::PutNumber("relayState", relay_activited);
            if (!timer_started_relay){
                startTimerRelay();  
            }
        }
        
    }
    time_t delay_relay = time(NULL) - start_relay;
    if (delay_relay >= 5){
        relay_solo.Set(0);
        std::cout <<"relay " << "off" << "\n";
    }

}

// ----------------------------------------------------------------------------
//

void Top::startTimerRelay(){

    if (!timer_started_relay){
        time(&start_relay);
        timer_started_relay = true;
    }

}



// ----------------------------------------------------------------------------
//

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

// ----------------------------------------------------------------------------
//

void Top::basculUp(){
    bascul_value = BASCUL_VALUE_AMP;
    basculHandler();
    setState(AmpState::BasculGoingUp);

}

// ----------------------------------------------------------------------------
//

void Top::basculIdle(){
    LB = m_controller.GetLeftBumper();
    bascul_down_speed = BASCUL_SPEED_DOWN_IDLE;
    if(LB && feeder_state == FeederState::Loaded){
        setState(AmpState::BasculUp);

    }
}

// ----------------------------------------------------------------------------
//

void Top::basculGoingUp(){
    if (angle_encoder_lancer <= BASCUL_VALUE_DEPLOY_FEEDER_AMP){
        feederUp(false);
    }
    if (bascul_target_position){
        setState(AmpState::FeederUp);
    }
}

// ----------------------------------------------------------------------------
//

void Top::feederUp(bool update_state){
    Feeder::feederEncoderReader();
    if(angle_encoder_feeder < AMP_POSITION_FEEDER_DOWN){ 
        if(feeder_speed != FEEDER_DOWN_SPEED){
            feeder_speed = FEEDER_DOWN_SPEED;
            setMotorFeeder(feeder_speed);

        }
        
    }else{
        feeder_speed = 0;
        setMotorFeeder(feeder_speed);
        if (update_state){
          setState(AmpState::Loaded);  
        }
        

    }
}

// ----------------------------------------------------------------------------
//

void Top::feederLoaded(){
    RT = m_controller.GetRightTriggerAxis();

    if(RT != 1){
        return;
    }

    intake_speed = INTAKE_PUSH_SPEED;
    setMotorIntake(intake_speed);
    setState(AmpState::Fire);
    
}

// ----------------------------------------------------------------------------
//

void Top::feederFire(){
    RT = m_controller.GetRightTriggerAxis();
    
    if(RT != 1){
        intake_speed = 0;
        setMotorIntake(intake_speed);
        setState(AmpState::FeederDown);
        
    }
}

// ----------------------------------------------------------------------------
//

void Top::feederDown(){
    Feeder::feederEncoderReader();
    if (angle_encoder_feeder <= FEEDER_VALUE_DEPLOY_BASCUL_AMP){
        basculDown(false);
    }
    if(angle_encoder_feeder > ENCODER_FEEDER_LANCER_VALUE){ 
        if(feeder_speed != FEEDER_UP_SPEED_AMP){
            feeder_speed = FEEDER_UP_SPEED_AMP;
            setMotorFeeder(feeder_speed);

        }
        
    }else{
        feeder_speed = 0;
        setMotorFeeder(feeder_speed);
        setState(AmpState::BasculDown);

    }
}

// ----------------------------------------------------------------------------
//

void Top::basculDown(bool update_state){
    bascul_value = MAX_VALUE_BASCUL;
    bascul_down_speed = BASCUL_AMP_GOING_DOWN_SPEED;
    basculHandler();
    if (update_state){
      setState(AmpState::BasculGoingDown);  
    }
    
}

// ----------------------------------------------------------------------------
//

void Top::basculGoingDown(){
    if (bascul_target_position){
        setState(AmpState::Idle);
    
    }
}

// ----------------------------------------------------------------------------
//

void Top::setState(AmpState state){
    //std::cout <<"amp_state " << state << "\n";
    frc::SmartDashboard::PutNumber("AmpState", state);
    amp_state = state; 
}

// ----------------------------------------------------------------------------
//