
#include "Base.h"
#include "Config.h"

// ----------------------------------------------------------------------------
//
void Base::RobotInit()
{
  m_autonomous = false;
  m_driveTrain.init(&m_nt, this);
  handleTopInit();
  m_driveTrain.handleTaskDriveTrainInit();
  m_nt.ntManagerInit();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Base::RobotPeriodic()
{
  #ifdef PATHPLANNER
    frc2::CommandScheduler::GetInstance().Run();
  #endif
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Base::DisabledInit() {

  m_nt.autonomous_pub.Set(0);
  m_nt.teleop_mode_pub.Set(0);
  m_nt.handleSubscriberTask();
  relay_activited = false;
}

// ----------------------------------------------------------------------------
//
void Base::AutonomousInit() {
  
  #ifdef PATHPLANNER
    m_autonomousCommand = m_container.getAutonomousCommand();

    if (m_autonomousCommand) {
      m_autonomousCommand->Schedule();
    }
  #endif

  m_nt.autonomous_pub.Set(1);
  m_nt.teleop_mode_pub.Set(0);

  first_note_shoot_append = false;
  second_note_shoot_append = false;
}

// ----------------------------------------------------------------------------
//
void Base::AutonomousPeriodic()
{
  if (!m_autonomous)
  {
    m_autonomous = true;
    logger.log(LL_NOTICE, "enter autonomous");
  }

  m_nt.handleSubscriberTask();

  if (m_nt.pos_value_auto == 2){
    //lance + take +shoot
    pos_auto_midle = true;
    if (!second_note_shoot_append){
      handleTopTaskAuto();
      logger.log(LL_NOTICE, "shoot midile note");
    }
    if (first_note_shoot_append && !second_note_shoot_append){
      m_driveTrain.handleTaskDriveTrainAuto();
      logger.log(LL_NOTICE, "drive midile autonomous");
    }

  }else{
    //shoot
    pos_auto_midle = false;
    handleTopTaskAuto();
    logger.log(LL_NOTICE, "single auto noto");
  }  

}

// ----------------------------------------------------------------------------
//
void Base::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  #ifdef PATHPLANNER
    if (m_autonomousCommand)
    {
      m_autonomousCommand->Cancel();
    }
  #endif

  m_nt.teleop_mode_pub.Set(1);
  m_nt.autonomous_pub.Set(0);
}

// ----------------------------------------------------------------------------
//
void Base::TeleopPeriodic()
{
  if (m_autonomous)
  {
    m_autonomous = false;
    logger.log(LL_NOTICE, "exit autonomous");
  }
  m_nt.handleSubscriberTask();
  m_driveTrain.handleTaskDriveTrainTeleop();
  handleTopTaskTeleop();
}

// ----------------------------------------------------------------------------
//

void Base::TestInit(){
  m_nt.teleop_mode_pub.Set(0);
  m_nt.autonomous_pub.Set(0);

}

// ----------------------------------------------------------------------------
//

void Base::TestPeriodic(){


}

// ----------------------------------------------------------------------------
//