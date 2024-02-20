#pragma once

#include"Config.h"

#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

#include <rev/CANSparkMax.h>

#include <cmath> 
#include <iostream>
using namespace std;

class Top {
 public:

 protected:

  void handleTopInit();
  void handleEncoderValue();
  void PositionFeeder();
  void PositionBasculTele();
  void handleTopTaskTeleop();
  void handlelancerSpeed();
  void intakeAnneau();

 private:
    frc::XboxController controller{CONTROLLER_PORT_NO};
    
    frc::DutyCycleEncoder m_dutyCycleEncoder_feeder{ENCODER_FEEDER};
    frc::DutyCycleEncoder m_dutyCycleEncoder_lancer{ENCODER_BASCUL};

    bool connect_encoder_feeder;
    bool connect_encoder_lancer;
    double angle_encoder_feeder;
    double angle_encoder_lancer;
    bool RB;
    bool LB;
    double RT;
    int pov;
    double bascul_value;
    double bascul_value_aprox;
    int feeder_possition_lancer_state;
    

    frc::DigitalInput anneau_limit_Switch{LIMIT_SWITCH};

    rev::CANSparkMax motor_feeder{MOTOR_FEEDER, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax motor_lancer_an_left{MOTOR_LANCER_AN_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax motor_lancer_an_right{MOTOR_LANCER_AN_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    
    rev::CANSparkMax motor_bascul_left{MOTOR_BASCUL_LEFT, rev::CANSparkMax::MotorType::kBrushless}; 
    rev::CANSparkMax motor_bascul_right{MOTOR_BASCUL_RIGHT, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_motor_intake{MOTOR_FEEDER_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
};  