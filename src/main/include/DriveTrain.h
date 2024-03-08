#pragma once

#include "Config.h"
#include "Nt_manager.h"
#include <FileLogger.h>

#include <frc/XboxController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <rev/CANSparkMax.h>
#include <frc/DriverStation.h>
#include <frc2/command/Subsystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <iostream>

using namespace std;

class DriveTrain : public frc2::Subsystem
{

public:
  bool init(NT_Manager *nt);
  void handleTaskDriveTrainAuto();
  void handleTaskDriveTrainTeleop();
  void handleTaskDriveTrainInit();

protected:
  tools::FileLogger logger{"DriveTrain"};

private:
  double x;
  double x_final;
  double y;
  double max_speed_tele;
  int pov;

  frc::Pose2d m_pose2d;
  frc::ChassisSpeeds m_speed;

  double motor_left_ID10;
  double motor_left_ID18;
  double motor_right_ID1;
  double motor_right_ID6;

  double left_speed_auto;
  double right_speed_auto;

  NT_Manager *m_nt;

  cs::UsbCamera camera1;

  frc::XboxController m_controller{CONTROLLER_PORT_NO};

  rev::CANSparkMax m_left_lead_motor{LEFT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_left_follow_motor{LEFT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_right_lead_motor{RIGHT_LEAD_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right_follow_motor{RIGHT_FOLLOW_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkRelativeEncoder m_encoder_left_lead{m_left_lead_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)};
  rev::SparkRelativeEncoder m_encoder_right_lead{m_right_lead_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)};

  frc::DifferentialDrive m_robotDrive{m_left_lead_motor, m_right_lead_motor};
  frc::DifferentialDriveKinematics m_kinematics{WHEEL_DISTANCE};

  void maxSpeedTele();
  void handleSensitivityStickLeftX();

  void handleDriveTeleop();
  void handleMotorBaseTemp();
  void handleTaskBaseTeleop();
  void baseInit();
  void handleDriveAuto();
  void handleShowAutoValue();

  frc::Pose2d getPose();
  void resetPose(frc::Pose2d pose);

  frc::ChassisSpeeds getSpeed();
  void setSpeed(frc::ChassisSpeeds speed);
  bool shouldFlipPath();
  bool isAllianceRed();
  int getAllianceLocation();
  void logSpeed(frc::ChassisSpeeds speed);
  void logPose(frc::Pose2d pose);
  double getWheelPWM(double speed);
};