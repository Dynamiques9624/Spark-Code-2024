
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

  setState(AutoState::Idle);
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
void Base::DisabledInit()
{

  m_nt.autonomous_pub.Set(0);
  m_nt.teleop_mode_pub.Set(0);
  m_nt.handleSubscriberTask();
  relay_activited = false;
}

// ----------------------------------------------------------------------------
//
void Base::AutonomousInit()
{

#ifdef PATHPLANNER
  m_autonomousCommand = m_container.getAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
#endif

  m_autonomous = true;
  logger.log(LL_NOTICE, "enter autonomous");

  m_nt.autonomous_pub.Set(1);
  m_nt.teleop_mode_pub.Set(0);

  first_note_shoot_append = false;

  timer_started_go_forward = false;

  setState(AutoState::FirstNote);
}

// ----------------------------------------------------------------------------
//
void Base::AutonomousPeriodic()
{
  time_t delay_go_forward;
  m_nt.handleSubscriberTask();

  switch (m_auto_state)
  {
  case Idle:
    break;
  
  case FirstNote:
    handleTopTaskAuto();
    if (first_note_shoot_append)
    {
      if (m_nt.pos_value_auto == 2)
      {
        setState(AutoState::DropFeeder);
      }
      else
      {
        setState(AutoState::Idle);
      }
    }
    break;

  case DropFeeder:

    if (getFeederState() == FeederState::Idle)
    {
      Feeder::setState(FeederState::GoDown);
    }
    else if (getFeederState() == FeederState::Suck)
    {
      setState(AutoState::SuckNote);
    }
    break;

  case SuckNote:
    m_driveTrain.handleTaskDriveTrainAuto();

    if (getFeederState() != FeederState::Suck)
    {
      setState(AutoState::GoForward);
    }
    break;

  case GoForward:
    startTimerGoForward();
    m_driveTrain.goForward();
    delay_go_forward = time(NULL) - start_go_forward;

    if (delay_go_forward >= 1)
    {
      m_driveTrain.stop();
      timer_started_propul = false;
      timer_started_fire = false;
      first_note_shoot_append = false;
      setState(AutoState::SecondNote);
    }
    break;

  case SecondNote:
    handleTopTaskAuto();
    if (first_note_shoot_append)
    {
      setState(AutoState::Idle);
    }
    break;

  default:
    break;
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

void Base::TestInit()
{
  m_nt.teleop_mode_pub.Set(0);
  m_nt.autonomous_pub.Set(0);
}

// ----------------------------------------------------------------------------
//

void Base::TestPeriodic()
{
}

// ----------------------------------------------------------------------------
//

void Base::setState(AutoState state)
{
  // std::cout <<"auto_state " << state << "\n";
  frc::SmartDashboard::PutNumber("AutoState", state);
  m_auto_state = state;
}

// ----------------------------------------------------------------------------
//

void Base::startTimerGoForward()
{
  if (!timer_started_go_forward)
  {
    time(&start_go_forward);
    timer_started_go_forward = true;
  }
}
