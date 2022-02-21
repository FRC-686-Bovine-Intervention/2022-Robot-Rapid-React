// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.modes.SimplePathFollowerAuto;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = AutoManager.getInstance();
  DriverInteraction driverInteraction = DriverInteraction.getInstance();

	AutoModeExecuter autoModeExecuter = null;
    
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
    // autoManager.init();
    if (autoModeExecuter != null)
    {
        autoModeExecuter.stop();
      }
      autoModeExecuter = null;
      
    autoModeExecuter = new AutoModeExecuter();
    autoModeExecuter.setAutoMode( new SimplePathFollowerAuto() );

    autoModeExecuter.start();    
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
