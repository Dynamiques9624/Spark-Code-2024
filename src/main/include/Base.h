#pragma once

#include"Config.h"
#include "Top.h"
#include "Feeder.h"
#include "Nt_manager.h"

#include <frc/TimedRobot.h>
#include <frc/AddressableLED.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>  
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

#include <rev/CANSparkMax.h>

#include <iostream>
using namespace std;

class Base : public frc::TimedRobot, Top {
 public:
  void RobotInit() override;
  void AutonomousPeriodic() override;
  void TeleopPeriodic() override;

 protected:
  void handleDriveTeleop();
  void handleMotorBaseTemp();
  void handleTaskBaseTeleop();
  void baseInit();
  void handleDriveAuto();
  void handleShowAutoValue();
  

 private:
  double x;
  double y;
  double max_speed_tele;
  int pov;

  double motor_left_ID10;
  double motor_left_ID18;
  double motor_right_ID1;
  double motor_right_ID6;

  double left_speed_auto;
  double right_speed_auto;

  cs::UsbCamera camera1;

  frc::XboxController controller{CONTROLLER_PORT_NO};  

  rev::CANSparkMax left_lead_motor{LEFT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};              
  rev::CANSparkMax left_follow_motor{LEFT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax right_lead_motor{RIGHT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_follow_motor{RIGHT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  frc::DifferentialDrive m_robotDrive{left_lead_motor, right_lead_motor};

};
