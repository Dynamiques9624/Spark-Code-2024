#pragma once

#include "Top.h"
#include "Feeder.h"
#include "Nt_manager.h"
#include "DriveTrain.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <math.h>
#include <iostream>

#include <RobotContainer.h>
#include <FileLogger.h>

using namespace std;

class Base : public frc::TimedRobot, Top
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;

protected:
  tools::FileLogger     logger{"Base"};
  
private:
  DriveTrain            m_driveTrain;
  bool                  m_autonomous;
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  RobotContainer        m_container;
};
