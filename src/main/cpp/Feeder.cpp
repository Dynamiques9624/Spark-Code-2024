#include "Feeder.h"

void Feeder::feederInit()
{
    m_dutyCycleEncoder_feeder.SetDistancePerRotation(360);
    connect_encoder_feeder = m_dutyCycleEncoder_feeder.IsConnected();
    frc::SmartDashboard::PutBoolean("encoder feeder", connect_encoder_feeder);

    motor_feeder.RestoreFactoryDefaults();

    feeder_state = Idle;
    feeder_speed = 0;
    intake_speed = 0;
    reset = false;
    shake_ring = true;
    shake_ring_enable = true;
    feeder_sucking = false;

    m_led.SetLength(KLENGTH);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

// ----------------------------------------------------------------------------
//

void Feeder::feederEncoderReader()
{
    angle_encoder_feeder = m_dutyCycleEncoder_feeder.GetDistance();
}

// ----------------------------------------------------------------------------
//

void Feeder::feederFireAuto(){
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
}

// ----------------------------------------------------------------------------
//

void Feeder::feederHandler()
{
    reset_button_x = m_controller.GetXButton();

    if (reset_button_x)
    {
        feederReset();
    }

    motorTemp();
    feederEncoderReader();
    frc::SmartDashboard::PutNumber("encoFeederVal", angle_encoder_feeder);
    frc::SmartDashboard::PutBoolean("feederSuck", feeder_sucking);

    switch (feeder_state)
    {
    case Idle:
        feederIdle();
        break;
    case GoDown:
        feederGoDown();
        break;
    case Suck:
        feederSuck();
        break;
    case GoUp:
        feederGoUp();
        break;
    case Loaded:
        feederLoaded();
        break;
    case Fire:
        feederFire();
        break;
    case PosAmp:
        feederPosAmp();
        break;
    }

    colorHandler();
    m_led.SetData(m_ledBuffer);
}

// ----------------------------------------------------------------------------
//

void Feeder::setState(FeederState state)
{
    // std::cout <<"state " << state << "\n";
    frc::SmartDashboard::PutBoolean("feederstate", state);
    feeder_state = state;
}

// ----------------------------------------------------------------------------
//

void Feeder::feederIdle()
{
    RB = m_controller.GetRightBumper();

    intake_speed = 0;
    motor_intake.Set(intake_speed);

    if (shake_ring_enable)
    {
        stopShakeRing();
    }

    if (anneau_limit_Switch.Get())
    {
        setState(Loaded);
    }

    if (RB)
    {
        setState(GoDown);
        feederGoDown();
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::feederReset()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder reset");
    #endif
    reset = true;
    setState(GoUp);
}

// ----------------------------------------------------------------------------
//

void Feeder::feederEject()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder edject");
    #endif
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
}

// ----------------------------------------------------------------------------
//

void Feeder::feederGoDown()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder goDown");
    #endif

    if (angle_encoder_feeder < ENCODER_FEEDER_TAKE_VALUE)
    {
        if (feeder_speed != FEEDER_DOWN_SPEED)
        {
            feeder_speed = FEEDER_DOWN_SPEED;
            motor_feeder.Set(feeder_speed);
        }
    }
    else
    {
        feeder_speed = 0;
        motor_feeder.Set(feeder_speed);
        setState(Suck);
        feederSuck();
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::feederSuck()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder suck");
    #endif
    feeder_sucking = true;
    eject_button_a = m_controller.GetAButton();

    if (eject_button_a)
    {
        feederEject();
        return;
    }

    if (intake_speed != INTAKE_SPEED_SUCK)
    {

        intake_speed = INTAKE_SPEED_SUCK;
        motor_intake.Set(intake_speed);
    }

    if (anneau_limit_Switch.Get())
    {
        m_nt.limit_switch_pub.Set(1);
        intake_speed = 0;
        motor_intake.Set(intake_speed);
        feeder_sucking = false;
        setState(GoUp);
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::feederGoUp()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder goUp");
    #endif

    if (shake_ring_enable)
    {
        if (shake_ring)
        {
            shakeRing();
        }
        else
        {
            startShakeRing();
        }
    }

    // arrive a destination?
    if (angle_encoder_feeder <= ENCODER_FEEDER_LANCER_VALUE)
    {
        feeder_speed = 0;
        motor_feeder.Set(feeder_speed);
        if (reset)
        {
            reset = false;
            setState(Idle);
        }
        else
        {
            setState(Loaded);
        }
        return;
    }

    if (feeder_speed < FEEDER_UP_MAX_SPEED)
    {
        double speed = ((ENCODER_FEEDER_TAKE_VALUE - angle_encoder_feeder) / ENCODER_FEEDER_SPEED_DIV) * FEEDER_SPEED_UP_INC;
        if (speed > FEEDER_UP_MAX_SPEED)
        {
            speed = FEEDER_UP_MAX_SPEED;
        }
        if (speed < FEEDER_SPEED_UP_INC)
        {
            speed = FEEDER_SPEED_UP_INC;
        }
        feeder_speed = speed;
        motor_feeder.Set(feeder_speed);
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::feederLoaded()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder loaded");
    #endif

    RT = m_controller.GetRightTriggerAxis();

    if (shake_ring_enable)
    {
        stopShakeRing();
    }

    if (RT != 1)
    {
        return;
    }
    intake_speed = INTAKE_PUSH_SPEED;
    motor_intake.Set(intake_speed);
    setState(Fire);
}

// ----------------------------------------------------------------------------
//

void Feeder::feederFire()
{
    #ifdef DEBUG_FEEDER
        logger.log(LL_NOTICE, "feeder fire");
    #endif

    RT = m_controller.GetRightTriggerAxis();

    if (RT != 1)
    {
        m_nt.limit_switch_pub.Set(0);
        intake_speed = 0;
        motor_intake.Set(intake_speed);
        setState(Idle);
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::feederPosAmp()
{

    if (feeder_speed < FEEDER_DOWN_AMP_MAX_SPEED)
    {
        double speed = ((ENCODER_FEEDER_TAKE_VALUE - angle_encoder_feeder) / ENCODER_FEEDER_SPEED_DIV) * FEEDER_SPEED_AMP_INC;
        if (speed > FEEDER_DOWN_AMP_MAX_SPEED)
        {
            speed = FEEDER_DOWN_AMP_MAX_SPEED;
        }
        if (speed < FEEDER_SPEED_AMP_INC)
        {
            speed = FEEDER_SPEED_AMP_INC;
        }
        feeder_speed = speed;
        motor_feeder.Set(feeder_speed);
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::startShakeRing()
{
    if (shake_ring_enable == false)
    {
        return;
    }

    if (angle_encoder_feeder <= ENCODER_FEEDER_STOP_SHAKE_VALUE)
    {
        return;
    }
    
    shake_ring = true;
    shake_ring_out = true;
    shake_ring_start = std::clock();
    // start ring out
    intake_speed = SHAKE_RING_INTAKE;
    motor_intake.Set(-intake_speed);
    //logger.log(LL_INFO, "start shake ring");
}

// ----------------------------------------------------------------------------
//

void Feeder::stopShakeRing()
{
    if (shake_ring_enable == false)
    {
        return;
    }

    if (anneau_limit_Switch.Get())
    {
        if (intake_speed != 0 || shake_ring)
        {
            shake_ring = false;
            intake_speed = 0;
            motor_intake.Set(intake_speed);

            //logger.log(LL_INFO, "stop shake ring");
        }
    }
}

// ----------------------------------------------------------------------------
//

void Feeder::shakeRing()
{
    if (shake_ring_enable == false || shake_ring == false)
    {
        return;
    }

    std::clock_t now = std::clock();
    double delay = diffclock(now, shake_ring_start);

    if (shake_ring_out)
    {
        if (delay >= SHAKE_RING_OUT_TIME)
        {
            //logger.log(LL_INFO, "shake ring in");
            // ring go in
            shake_ring_out = false;
            shake_ring_start = now;
            intake_speed = SHAKE_RING_INTAKE;
            motor_intake.Set(intake_speed);
        }
    }
    else
    {
        //  check limit switch
        if (anneau_limit_Switch.Get() || delay >= SHAKE_RING_OUT_TIME)
        {
            if (angle_encoder_feeder <= ENCODER_FEEDER_STOP_SHAKE_VALUE)
            {
                stopShakeRing();
            }
            else
            {
                //logger.log(LL_INFO, "shake ring out");
                startShakeRing();
            }
        }
    }
}

// ----------------------------------------------------------------------------
//

double Feeder::diffclock(std::clock_t clock1, std::clock_t clock2)
{
    double diffticks = clock1 - clock2;
    double diffms = (diffticks) / (CLOCKS_PER_SEC / 1000);
    return diffms;
}

// ----------------------------------------------------------------------------
//

void Feeder::setMotorFeeder(double feeder_speed)
{
    motor_feeder.Set(feeder_speed);
}

// ----------------------------------------------------------------------------
//

void Feeder::setMotorIntake(double intake_speed)
{
    motor_intake.Set(intake_speed);
}

// ----------------------------------------------------------------------------
//

void Feeder::motorTemp()
{
    temp_m_feeder = motor_feeder.GetMotorTemperature();
    temp_m_intake = motor_intake.GetMotorTemperature();

    frc::SmartDashboard::PutNumber("tempMfeeder", temp_m_feeder);
    frc::SmartDashboard::PutNumber("tempMintake", temp_m_intake);
}

// ----------------------------------------------------------------------------
//

void Feeder::colorHandler()
{
    for (int i = 0; i < KLENGTH; i++)
    {
        if (feeder_state == Loaded)
        {
            // grean
            m_ledBuffer[i].SetHSV(62, 237, 108);
        }
        if (feeder_state == Suck)
        {
            // blue
            m_ledBuffer[i].SetHSV(61, 73, 252);
        }
        if (feeder_state == Idle || feeder_state == GoDown || feeder_state == GoUp)
        {
            // red
            m_ledBuffer[i].SetHSV(252, 0, 0);
        }
    }
}

// ----------------------------------------------------------------------------
//
