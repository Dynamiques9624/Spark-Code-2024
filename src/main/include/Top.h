#pragma once

#include"Config.h"
#include "Feeder.h"

#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>

#include <cmath> 
#include <time.h>
#include <iostream>
//using namespace std;

class Top : protected Feeder{
 public:

 protected:
  enum AmpState{
    Idle, BasculUp, BasculGoingUp, FeederUp, Loaded, Fire, FeederDown, BasculDown, BasculGoingDown
  };
  AmpState amp_state;

  void setState(AmpState state);

  void handleTopInit();
  void handleEncoderValue();
  void handleMotorTemp();
  
  void ampHandler();
  void PositionBascul();
  void basculHandler();
  void handleTopTaskTeleop();
  void handleTopTaskAuto();
  void handlelancerSpeed();
  void lanceurAuto();
  

  double bascul_value;

 private:

    bool bascul_target_position;

    bool connect_encoder_lancer;

    bool y_button_handler;

    double y_button;

    double feeder_speed;
    double intake_speed;
    
    double angle_encoder_lancer;
    bool RB;
    bool LB;
    double RT;
    double LT;
    int pov;
    
    double bascul_value_aprox;

    bool set_predefine_loaded;
    
    double temp_m_lancer_left;
    double temp_m_lancer_right;
    double temp_m_bascul_left;
    double temp_m_bascul_right;

    frc::XboxController m_controller{CONTROLLER_PORT_NO};
        
    frc::DutyCycleEncoder m_dutyCycleEncoder_lancer{ENCODER_BASCUL};

    rev::CANSparkMax motor_lancer_an_left{MOTOR_LANCER_AN_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax motor_lancer_an_right{MOTOR_LANCER_AN_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    
    rev::CANSparkMax motor_bascul_left{MOTOR_BASCUL_LEFT, rev::CANSparkMax::MotorType::kBrushless}; 
    rev::CANSparkMax motor_bascul_right{MOTOR_BASCUL_RIGHT, rev::CANSparkMax::MotorType::kBrushless};


    void yButtonHandler();

    void basculUp();
    void basculIdle();
    void basculGoingUp();
    void feederUp(bool update_state = true);
    void feederLoaded();
    void feederFire();
    void feederDown();
    void basculDown(bool update_state = true);
    void basculGoingDown();
};  