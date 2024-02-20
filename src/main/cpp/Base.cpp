// base est le code maine 

#include "Base.h"

void Base::RobotInit() {
  handleTopInit();

  m_left_follow_motor.RestoreFactoryDefaults();
  m_left_lead_motor.RestoreFactoryDefaults();
  m_right_follow_motor.RestoreFactoryDefaults();
  m_right_lead_motor.RestoreFactoryDefaults(); 

  m_left_follow_motor.Follow(m_left_lead_motor);
  m_right_follow_motor.Follow(m_right_lead_motor);

  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();

  max_speed_tele = 1;
}

void Base::AutonomousPeriodic(){



}

void Base::TeleopPeriodic() { 
  
  handleDriveTeleop();
  handleTopTaskTeleop();

  Color();
  m_led.SetData(m_ledBuffer);
}


//fonction pour la base

void Base::Color() {

  for (int i = 0; i < kLength; i++) {
    if (x > 0.1){
      //color backward
      m_ledBuffer[i].SetHSV(242, 51, 51);
    } 
    if (x < -0.1){
      //color forward
      m_ledBuffer[i].SetHSV(24, 35, 245);
    }
    else{
      m_ledBuffer[i].SetHSV(28, 255, 255);
    }    
  }

}

void Base::handleDriveTeleop(){

  x = controller.GetLeftX();
  y = controller.GetLeftY();
  pov = controller.GetPOV();

  if (pov == 90 && max_speed_tele < 1){
    max_speed_tele += 0.01;
  }
  if(pov == 270 && max_speed_tele > 0.1){
    max_speed_tele -=0.01;
  }
  //std::cout<<"anglevalue" <<max_speed_tele <<"\n";

  if (x > max_speed_tele){
      x = max_speed_tele;
  }
  if (y > max_speed_tele){
      y = max_speed_tele;
  }
  if (x < -max_speed_tele){
      x = -max_speed_tele;
  }
  if (y < -max_speed_tele){
      y = -max_speed_tele;
  }
  frc::SmartDashboard::PutNumber("maxSpeed", max_speed_tele);
  m_robotDrive.ArcadeDrive(x,y);

}
