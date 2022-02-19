// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Auto.AutoManager;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.Subsystems.Drive;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = AutoManager.getInstance();
  DriverInteraction driverInteraction = DriverInteraction.getInstance();

  @Override
  public void robotInit() {
    subsystemManager.init();
    autoManager.InitChoices();
    LoopController.getInstance().register(Drive.getInstance().getVelocityPIDLoop());
    LoopController.getInstance().register(DriveLoop.getInstance());
  }

  @Override
  public void robotPeriodic() {subsystemManager.updateShuffleboard(); LoopController.getInstance().run();}

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
    LoopController.getInstance().start();
  }

  @Override
  public void teleopPeriodic() {
    subsystemManager.run();
    driverInteraction.run();
  }

  @Override
  public void disabledInit() {LoopController.getInstance().start();}

  @Override
  public void disabledPeriodic() {
    subsystemManager.disable();
  }

  @Override
  public void testInit() {LoopController.getInstance().start();}

  @Override
  public void testPeriodic() {
    subsystemManager.run();
  }
}
