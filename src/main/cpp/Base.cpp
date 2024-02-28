
#include "Base.h"

void Base::RobotInit() {
  handleTopInit();
  baseInit();

}

void Base::AutonomousPeriodic(){
  autonomous_pub.Set(1);
  handleTopTaskAuto();
  handleShowAutoValue();
  handleSubscriberTask();
  handleDriveAuto();

}

void Base::TeleopPeriodic() { 
  autonomous_pub.Set(0);
  handleTaskBaseTeleop();
  handleTopTaskTeleop();

}

//Base code------------------------------------------------------------------------------------------------------------


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

void Base::handleMotorBaseTemp(){
  motor_left_ID10 = left_lead_motor.GetMotorTemperature();
  motor_left_ID18 = left_follow_motor.GetMotorTemperature();
  motor_right_ID1 = right_lead_motor.GetMotorTemperature();
  motor_right_ID6 = right_follow_motor.GetMotorTemperature();

  frc::SmartDashboard::PutNumber("mLeftID10", motor_left_ID10);
  frc::SmartDashboard::PutNumber("mLeftID18", motor_left_ID18);
  frc::SmartDashboard::PutNumber("mRightID1", motor_right_ID1);
  frc::SmartDashboard::PutNumber("mRightID6", motor_right_ID6);
}

void Base::handleTaskBaseTeleop(){
  handleDriveTeleop();
  handleMotorBaseTemp();
}


void Base::baseInit(){
  camera1 = frc::CameraServer::StartAutomaticCapture(0);

  left_follow_motor.RestoreFactoryDefaults();
  left_lead_motor.RestoreFactoryDefaults();
  right_follow_motor.RestoreFactoryDefaults();
  right_lead_motor.RestoreFactoryDefaults(); 

  left_follow_motor.Follow(left_lead_motor);
  right_follow_motor.Follow(right_lead_motor);
  
  max_speed_tele = 1;

}

void Base::handleDriveAuto(){

  left_speed_auto = left_wheel_speed_percent/100;
  right_speed_auto = right_wheel_speed_percent/100;
  
  if(left_speed_auto > DRIVE_MAX_SPEED_AUTO){
    left_speed_auto = DRIVE_MAX_SPEED_AUTO;
  }
  if(right_speed_auto > DRIVE_MAX_SPEED_AUTO){
    right_speed_auto = DRIVE_MAX_SPEED_AUTO;
  }
  if(left_speed_auto < -DRIVE_MAX_SPEED_AUTO){
    left_speed_auto = -DRIVE_MAX_SPEED_AUTO;
  }
  if(right_speed_auto < -DRIVE_MAX_SPEED_AUTO){
    right_speed_auto = -DRIVE_MAX_SPEED_AUTO;
  }
  
  left_lead_motor.Set(left_speed_auto);
  right_lead_motor.Set(right_speed_auto);

}

void Base::handleShowAutoValue(){

  frc::SmartDashboard::PutNumber("ringDetected", ring_detected);
  frc::SmartDashboard::PutNumber("tagDetected", tag_detected);
  frc::SmartDashboard::PutNumber("tagID", tag_id);
}

