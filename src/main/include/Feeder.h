#pragma once

#include "Nt_manager.h"
#include "Config.h"
#include <FileLogger.h>
#include <array>

#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/AddressableLED.h>

#include <rev/CANSparkMax.h>

#include <cmath> 
#include <iostream>
using namespace std;

class Feeder{
 public:
    enum FeederState{
        Idle, GoDown , Suck, GoUp, Loaded, Fire, PosAmp, GoAmp
    };

    Feeder::FeederState getFeederState() const {return feeder_state;}
 protected:
    
    FeederState feeder_state;

    NT_Manager m_nt;

    double angle_encoder_feeder;

    void feederInit();
    void feederHandler();
    void colorHandler();
    void setState(FeederState state);
    void feederEncoderReader();
    void setMotorFeeder(double feeder_speed);
    void setMotorIntake(double intake_speed);
    void feederFireAuto();
    
 private:
    bool connect_encoder_feeder;

    bool shake_ring;
    bool shake_ring_enable;
    bool shake_ring_out;
    std::clock_t shake_ring_start;
    
 
    bool RB;
    double RT;
    bool reset_button_x;
    bool eject_button_a;

    bool reset;
    
    double feeder_speed;
    double intake_speed;

    bool feeder_sucking;

    double temp_m_feeder;
    double temp_m_intake;

    tools::FileLogger     logger{"Feeder"};

    frc::XboxController m_controller{CONTROLLER_PORT_NO};
    
    frc::DutyCycleEncoder m_dutyCycleEncoder_feeder{ENCODER_FEEDER};

    frc::AddressableLED m_led{LED_STRIP_PWM};
    std::array<frc::AddressableLED::LEDData, KLENGTH>
        m_ledBuffer; 
    frc::DigitalInput anneau_limit_Switch{LIMIT_SWITCH};

    rev::CANSparkMax motor_feeder{MOTOR_FEEDER, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax motor_intake{MOTOR_FEEDER_INTAKE, rev::CANSparkMax::MotorType::kBrushless};

    void feederIdle();
    void feederGoDown();
    void feederSuck();
    void feederGoUp();
    void feederLoaded();
    void feederFire();
    void feederReset();
    void feederEject();
    void feederPosAmp();
    void motorTemp();
    void startShakeRing();
    void stopShakeRing();
    void shakeRing();
    double diffclock(std::clock_t clock1, std::clock_t clock2);
};  