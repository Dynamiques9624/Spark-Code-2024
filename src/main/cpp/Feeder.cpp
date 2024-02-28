#include "Feeder.h"

void Feeder::feederInit(){
    ntManagerInit();

    m_dutyCycleEncoder_feeder.SetDistancePerRotation(360);
    connect_encoder_feeder = m_dutyCycleEncoder_feeder.IsConnected();
    frc::SmartDashboard::PutBoolean("encoder feeder", connect_encoder_feeder);

    motor_feeder.RestoreFactoryDefaults();

    feeder_state = Idle;
    feeder_speed = 0;
    intake_speed = 0;
    reset = false;

    m_led.SetLength(KLENGTH);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

}

void Feeder::feederEncoderReader(){
    angle_encoder_feeder = m_dutyCycleEncoder_feeder.GetDistance();
}


void Feeder::feederHandler(){
    reset_button_x = controller.GetXButton();

    if (reset_button_x){
        feederReset();
    }
    
    motorTemp();
    feederEncoderReader();
    frc::SmartDashboard::PutNumber("encoFeederVal", angle_encoder_feeder);

    switch (feeder_state){
    case Idle:
        feederIdle();
        break;
    case GoDown:
        feederGoDown();
        break;
    case Suck:
        feederSuck();
        break;
    case GoUp:
        feederGoUp();
        break;
    case Loaded:
        feederLoaded();
        break;
    case Fire:
        feederFire();
        break;
    case PosAmp:
        feederPosAmp();
        break;
    }
    colorHandler();
    m_led.SetData(m_ledBuffer);

}

void Feeder::setState(FeederState state){
    std::cout <<"state " << state << "\n";
    frc::SmartDashboard::PutBoolean("feederstate", state);
    feeder_state = state; 
}

void Feeder::feederIdle(){
    RB = controller.GetRightBumper();

    intake_speed = 0;
    motor_intake.Set(intake_speed);

    if (anneau_limit_Switch.Get()){
        setState(Loaded);
    }

    if(RB){
        setState(GoDown);
        feederGoDown();
    }

}

void Feeder::feederReset(){
    reset = true;
    setState(GoUp);
}

void Feeder::feederEject(){
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
}

void Feeder::feederGoDown(){
    
    if(angle_encoder_feeder < ENCODER_FEEDER_TAKE_VALUE){ 
        if(feeder_speed != FEEDER_DOWN_SPEED){
            feeder_speed = FEEDER_DOWN_SPEED;
            motor_feeder.Set(feeder_speed);
        }
        
    }else{
        feeder_speed = 0;
        motor_feeder.Set(feeder_speed);
        setState(Suck);
        feederSuck();
    }

}


void Feeder::feederSuck(){
    eject_button_a = controller.GetAButton();

    if (eject_button_a){
        feederEject();
        return;
    }

    if(intake_speed != INTAKE_SPEED_SUCK){
        
        intake_speed = INTAKE_SPEED_SUCK;
        motor_intake.Set(intake_speed);   
    }
        
    if(anneau_limit_Switch.Get()){
        intake_speed = 0;
        motor_intake.Set(intake_speed);
        setState(GoUp);
    }

}

void Feeder::feederGoUp(){
    
    if(angle_encoder_feeder <= ENCODER_FEEDER_LANCER_VALUE){
        feeder_speed = 0;
        motor_feeder.Set(feeder_speed);
        if (reset) {
            reset = false;
            setState(Idle);
        } else {
            setState(Loaded);
        }
        return;
    }

    if(feeder_speed < FEEDER_UP_MAX_SPEED){
        double speed = ((ENCODER_FEEDER_TAKE_VALUE - angle_encoder_feeder)/ENCODER_FEEDER_SPEED_DIV)*FEEDER_SPEED_UP_INC;
        if(speed > FEEDER_UP_MAX_SPEED){
            speed = FEEDER_UP_MAX_SPEED;
        }
        if (speed < FEEDER_SPEED_UP_INC){
            speed = FEEDER_SPEED_UP_INC;
        }
        feeder_speed = speed;
        motor_feeder.Set(feeder_speed);
        
    }

}

void Feeder::feederLoaded(){
    RT = controller.GetRightTriggerAxis();

    if(RT != 1){
        return;
    }
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
    setState(Fire);

}

void Feeder::feederFire(){
    RT = controller.GetRightTriggerAxis();
    
    if(RT != 1){
        intake_speed = 0;
        motor_intake.Set(intake_speed);
        setState(Idle);
    }
}

void Feeder::feederFireAuto(){
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
}

void Feeder::feederPosAmp(){

    if(feeder_speed < FEEDER_DOWN_AMP_MAX_SPEED){
        double speed = ((ENCODER_FEEDER_TAKE_VALUE - angle_encoder_feeder)/ENCODER_FEEDER_SPEED_DIV)*FEEDER_SPEED_AMP_INC;
        if(speed > FEEDER_DOWN_AMP_MAX_SPEED){
            speed = FEEDER_DOWN_AMP_MAX_SPEED;
        }
        if (speed < FEEDER_SPEED_AMP_INC){
            speed = FEEDER_SPEED_AMP_INC;
        }
        feeder_speed = speed;
        motor_feeder.Set(feeder_speed);
        
    }
    
}

void Feeder::setMotorFeeder(double feeder_speed){
    motor_feeder.Set(feeder_speed);
}

void Feeder::setMotorIntake(double intake_speed){
    motor_intake.Set(intake_speed);
}

void Feeder::motorTemp(){
    temp_m_feeder = motor_feeder.GetMotorTemperature();
    temp_m_intake = motor_intake.GetMotorTemperature();

    frc::SmartDashboard::PutNumber("tempMfeeder", temp_m_feeder);
    frc::SmartDashboard::PutNumber("tempMintake", temp_m_intake);
}

void Feeder:: colorHandler(){
    for (int i = 0; i < KLENGTH; i++) {
        if (feeder_state == Loaded){
            //grean
            m_ledBuffer[i].SetHSV(62, 237, 108);
        } 
        if (feeder_state == Suck){
            //blue
            m_ledBuffer[i].SetHSV(61, 73, 252);
        }
        if (feeder_state == Idle || feeder_state == GoDown || feeder_state == GoUp){
            //red
            m_ledBuffer[i].SetHSV(252, 0, 0);
        }
    }
}

