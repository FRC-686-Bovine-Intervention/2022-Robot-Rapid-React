// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.ArmSubsystem;


public class Robot extends TimedRobot {

  Joystick joystick = new Joystick(0);
  ArmSubsystem arm = new ArmSubsystem();

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    arm.zeroSensors();    
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    arm.zeroSensors();
  }

  @Override
  public void teleopPeriodic() {
    double armCtrl = joystick.getRawAxis(1);
    if (Math.abs(armCtrl)>0.1)
    {
      armCtrl *= 0.3;
      arm.manualControl(armCtrl);
    }

    if (joystick.getRawButtonPressed(1)) {
      arm.setTarget(ArmSubsystem.kGroundPositionDegrees);
    }
    
    if (joystick.getRawButtonPressed(2)) {
      arm.setTarget(ArmSubsystem.kScoringPositionDegrees);
    }
    
    if (joystick.getRawButtonPressed(4)) {
      arm.setTarget(ArmSubsystem.kCalibrationPositionDegrees);
    }
    
    arm.onLoop();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    
  }

}
