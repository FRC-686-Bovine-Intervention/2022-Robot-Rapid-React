// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Auto.AutoManager;
import frc.robot.Subsystems.SubsystemManager;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = AutoManager.getInstance();
  DriverInteraction driverInteraction = DriverInteraction.getInstance();

  @Override
  public void robotInit() {
    subsystemManager.init();
    autoManager.InitChoices();
  }

  @Override
  public void robotPeriodic() {subsystemManager.updateShuffleboard();}

  @Override
  public void autonomousInit() {
    autoManager.init();
  }

  @Override
  public void autonomousPeriodic() {
    autoManager.run();
    subsystemManager.run();
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    subsystemManager.run();
    driverInteraction.run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    subsystemManager.runTestMode();
  }
}
