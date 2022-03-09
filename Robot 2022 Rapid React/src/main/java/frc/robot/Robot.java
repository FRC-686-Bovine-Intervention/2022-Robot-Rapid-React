// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.auto.AutoManager;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = AutoManager.getInstance();
  DriverInteraction driverInteraction = DriverInteraction.getInstance();

  private NetworkTableEntry headingEntry = Shuffleboard.getTab("Robot Status").add("Heading Degrees", -999999).getEntry();
  private NetworkTableEntry distanceEntry = Shuffleboard.getTab("Robot Status").add("Distance", -999999).getEntry();
  private NetworkTableEntry poseEntry = Shuffleboard.getTab("Robot Status").add("Pose Entry", "not updating").getEntry();
  private double startingDistance;

  private double getDistance()
    {
        return (DriveState.getInstance().getLeftDistanceInches() + DriveState.getInstance().getRightDistanceInches())/2;
    }

  @Override
  public void robotInit() {
    subsystemManager.init();
    autoManager.InitChoices();
    LoopController.getInstance().register(Drive.getInstance().getVelocityPIDLoop());
    LoopController.getInstance().register(DriveLoop.getInstance());
    LoopController.getInstance().register(RobotStateLoop.getInstance());
  }

  @Override
  public void robotPeriodic() {subsystemManager.updateShuffleboard(); LoopController.getInstance().run();
    distanceEntry.setDouble(getDistance() - startingDistance);
    headingEntry.setDouble(RobotState.getInstance().getLatestFieldToVehicle().getHeadingDeg());
    poseEntry.setString(RobotState.getInstance().getLatestFieldToVehicle().toString());
  }

  @Override
  public void autonomousInit() {
    startingDistance = getDistance();
    autoManager.init();
  }

  @Override
  public void autonomousPeriodic() {
    subsystemManager.run();
  }

  @Override
  public void teleopInit() {
    LoopController.getInstance().start();
    driverInteraction.init();
  }

  @Override
  public void teleopPeriodic() {
    driverInteraction.run();
    subsystemManager.run();
  }

  @Override
  public void disabledInit() {LoopController.getInstance().start(); autoManager.stop();}

  @Override
  public void disabledPeriodic() {
    subsystemManager.disable();
  }

  @Override
  public void testInit() {LoopController.getInstance().start();}

  @Override
  public void testPeriodic() {
    subsystemManager.runTestMode();
  }
}
