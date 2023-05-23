// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>
#include <array>
#include <vector>
#include <memory>
#include <cstdint>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

constexpr double STICK_DEADZONE = 0.2;

static std::uint64_t GetEpoch()
{
	using namespace std::chrono;
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

/* Mechanum talon motors base controller */
class TalonMechanum final
{
public:
  /* Pass ids in order from top left to down right e.g. (top_left, bottom_left, top_right, bottom_right) */
  TalonMechanum(frc::XboxController& controller, const std::array<int, 4>& ids)
    : controller(controller)
  {
    for (int i{ 0 }; i < 4; i++)
      motors.push_back(std::make_unique<TalonSRX>(ids[i]));
  }

  void Reset()
  {
    if (is_reset)
      return;

    for (int i{0}; i < 4; i++)
      motors[i]->Set(mode, 0);

    is_reset = true;
  }

  void Drive()
  {
    static double ratio{ 0.5 };
    double left_stick_x{ controller.GetLeftX()*ratio };
    double left_stick_y{ controller.GetLeftY()*ratio };
    double right_stick_x{ controller.GetRightX()*ratio };

    if (std::abs(right_stick_x) > STICK_DEADZONE)
    {
      // Set left motors
      motors[0]->Set(mode, -right_stick_x);
      motors[1]->Set(mode, -right_stick_x);

      // Set right motors
      motors[2]->Set(mode, -right_stick_x);
      motors[3]->Set(mode, -right_stick_x);
    }
    else if (std::abs(left_stick_x) > STICK_DEADZONE || std::abs(left_stick_y) > STICK_DEADZONE)
    {
      bool is_x{ std::abs(left_stick_x) > STICK_DEADZONE ? true : false }; // DRY

      if (is_x)
      {
        // Set left motors
        motors[0]->Set(mode, -left_stick_x);
        motors[1]->Set(mode, left_stick_x);

        // Set right motors
        motors[2]->Set(mode, -left_stick_x);
        motors[3]->Set(mode, left_stick_x); 
      }
      else
      {
        // Set left motors
        motors[0]->Set(mode, left_stick_y);
        motors[1]->Set(mode, left_stick_y);

        // Set right motors
        motors[2]->Set(mode, -left_stick_y);
        motors[3]->Set(mode, -left_stick_y); 
      }
    }
    else 
    {
      Reset();
      return;
    }

    is_reset = false;
  }
private:
  frc::XboxController& controller;

  TalonSRXControlMode mode = TalonSRXControlMode::PercentOutput;
  std::vector<std::unique_ptr<TalonSRX>> motors;

  bool is_reset{false};
};

class Robot : public frc::TimedRobot 
{
 public:
  void TeleopPeriodic() override 
  {   
    static double pov_ratio = 0.3;
    
    //
    // Global controls
    //

    mechanum_drive->Drive();

    switch (controller.GetPOV())
    {
    case 0:
      shooter_motor.Set(pov_ratio);
      break;
    case 180:
      shooter_motor.Set(-pov_ratio);
      break;
    default:
      shooter_motor.Set(0);
    }

    if (controller.GetLeftStickButtonPressed())
    {
      /*
      horn_one.Set(true);
      horn_two.Set(true);
      */
    }
    else
    {
      /*
      horn_one.Set(false);
      horn_two.Set(false);
      */
    }

    //
    // Pneumatic controls
    //

    if (controller.GetXButton() && !shooter_wait_charge)
    {
      shooter_charging = !shooter_charging;
      left_solenoid.Set(shooter_charging);
      
      shooter_wait_charge = true;
      last_epoch_charge = GetEpoch();
    }
    else if (shooter_wait_charge)
    {
      std::uint64_t curr_epoch = GetEpoch();

      if (curr_epoch - last_epoch_charge >= 250)
      {
        //left_solenoid.Set(false);
        shooter_wait_charge = false;
      }
    }

    if (controller.GetAButton() && !shooter_wait_shoot)
    {
      if (shooter_charging)
      {
        shooter_charging = false;
        left_solenoid.Set(false);
      }

      right_solenoid.Set(true);
      shooter_wait_shoot = true;
      last_epoch_shoot = GetEpoch();
    }
    else if (shooter_wait_shoot)
    {
      std::uint64_t curr_epoch = GetEpoch();

      if (curr_epoch - last_epoch_shoot >= 1500)
      {
        right_solenoid.Set(false);
        shooter_wait_shoot = false;
      }
    }

    dashboard.SetDefaultBoolean("Shooter charging", shooter_charging);
  }

  void RobotInit() override 
  {
    dashboard.init();
    
    const std::array<int, 4> ids = {5, 3, 4, 2};
    mechanum_drive = std::make_unique<TalonMechanum>(controller, ids);
    
    left_solenoid.Set(false);
    right_solenoid.Set(false);
  }
 private:
  frc::XboxController controller{0};
  rev::CANSparkMax shooter_motor{30, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  std::unique_ptr<TalonMechanum> mechanum_drive;

  frc::Solenoid left_solenoid{frc::PneumaticsModuleType::CTREPCM, 1};
  frc::Solenoid right_solenoid{frc::PneumaticsModuleType::CTREPCM, 0};

  //frc::Solenoid horn_one{frc::PneumaticsModuleType::CTREPCM, 2};
  //frc::Solenoid horn_two{frc::PneumaticsModuleType::CTREPCM, 3};

  bool shooter_charging{false};
  bool shooter_wait_charge{false};
  bool shooter_wait_shoot{false};
  std::uint64_t last_epoch_shoot{0};
  std::uint64_t last_epoch_charge{0};

 frc::SmartDashboard dashboard;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif