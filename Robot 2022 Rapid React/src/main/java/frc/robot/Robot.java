// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {

  Joystick driverController = new Joystick(0);

  WPI_TalonFX driveLeftA = new WPI_TalonFX(Constants.kLeftMasterID);
  WPI_TalonFX driveLeftB = new WPI_TalonFX(Constants.kLeftSlaveID);
  WPI_TalonFX driveRightA = new WPI_TalonFX(Constants.kRightMasterID);
  WPI_TalonFX driveRightB = new WPI_TalonFX(Constants.kRightSlaveID);

  WPI_TalonFX arm = new WPI_TalonFX(Constants.kArmMotorID);
  VictorSPX intake = new VictorSPX(Constants.kRollerMotorID);
  
  //Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.4;

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.35;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0;

  double autoStart = 0;
  boolean goForAuto = false;


  @Override
  public void robotInit() {
    driveLeftA.configFactoryDefault();
    driveLeftB.configFactoryDefault();
    driveRightA.configFactoryDefault();
    driveRightB.configFactoryDefault();

    driveLeftA.setInverted(TalonFXInvertType.CounterClockwise);
    driveLeftB.setInverted(TalonFXInvertType.CounterClockwise);
    driveRightA.setInverted(TalonFXInvertType.Clockwise);
    driveRightB.setInverted(TalonFXInvertType.Clockwise);

    driveLeftB.follow(driveLeftA);
    driveRightB.follow(driveRightA);

    driveLeftA.setNeutralMode(NeutralMode.Coast);
    driveLeftB.setNeutralMode(NeutralMode.Coast);
    driveRightA.setNeutralMode(NeutralMode.Coast);
    driveRightB.setNeutralMode(NeutralMode.Coast);

    arm.configFactoryDefault();
    arm.setInverted(TalonFXInvertType.CounterClockwise);
    arm.setNeutralMode(NeutralMode.Brake);

    intake.configFactoryDefault();
    intake.setInverted(false);
    intake.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(0);
    
    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftPower *= 0.3;
    driveRightPower *= 0.3;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);

    //Intake controls
    if(driverController.getRawButton(1)){
      intake.set(VictorSPXControlMode.PercentOutput, 0.7);
    }
    else if(driverController.getRawButton(3)){
      intake.set(VictorSPXControlMode.PercentOutput, -0.9);
    }
    else{
      intake.set(VictorSPXControlMode.PercentOutput, 0);
    }

    //Arm Controls
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldDown);
      }
    }

    // if(driverController.getRawButtonPressed(6) && !armUp){
    if(driverController.getRawAxis(5) < -0.5 && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    // else if(driverController.getRawButtonPressed(8) && armUp){
    else if(driverController.getRawAxis(5) > 0.5 && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    }  


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
