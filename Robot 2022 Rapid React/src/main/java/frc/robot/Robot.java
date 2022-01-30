// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Auto.AutoManager;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.Subsystems.DriverInteraction;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = new AutoManager();

  @Override
  public void robotInit() {
    subsystemManager.init();
    autoManager.InitChoices();
  }

  @Override
  public void robotPeriodic() {subsystemManager.updateSmartDashboard();}

  @Override
  public void autonomousInit() {
    autoManager.init();
  }

  @Override
  public void autonomousPeriodic() {
    autoManager.run();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    DriverInteraction.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
