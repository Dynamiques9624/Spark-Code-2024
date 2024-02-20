#pragma once

#include"Config.h"
#include "Top.h"

#include <array>

#include <frc/TimedRobot.h>
#include <frc/AddressableLED.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>  
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>

#include <iostream>
using namespace std;

class Base : public frc::TimedRobot, Top {
 public:
  void RobotInit() override;
  void AutonomousPeriodic() override;
  void TeleopPeriodic() override;


 protected:
  void handleDriveTeleop();
  void Color();

 private:

  static constexpr int kLength = 60;
  double x;
  double y;
  double max_speed_tele;
  int pov;
  
  frc::AddressableLED m_led{LED_STRIP_PWM};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer; 

  frc::XboxController controller{CONTROLLER_PORT_NO};  

  rev::CANSparkMax m_left_lead_motor{LEFT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};              
  rev::CANSparkMax m_left_follow_motor{LEFT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_right_lead_motor{RIGHT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right_follow_motor{RIGHT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  

  frc::DifferentialDrive m_robotDrive{m_left_lead_motor, m_right_lead_motor};

};
