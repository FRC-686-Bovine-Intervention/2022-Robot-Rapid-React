// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Robot extends TimedRobot {

  final int armMotorCanId = 6;
  Joystick joystick = new Joystick(0);

  WPI_TalonFX armMotor = new WPI_TalonFX(Constants.kArmMotorID);

  static final double armMotorStallCurrentThrehold = 60.0;

  boolean calibrated = false;
  double armCalibrationPositionDegrees = 112;
  double armScoringPositionDegrees = 90.0;
  double armGroundPositionDegrees = 5.0;

  static final double armGearRatio = 16.0 * 48.0/12.0;  // 16 in gearbox, 48t:12t sprockets
  static final double kEncoderUnitsPerRev = 2048 * armGearRatio;
  static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;

  ShuffleboardTab tab = Shuffleboard.getTab("ArmDebug");
  NetworkTableEntry calibratedEntry = tab.add("calibrated", false).getEntry();
  NetworkTableEntry joystickEntry = tab.add("joystick", 0.0).getEntry();
  NetworkTableEntry voltageEntry = tab.add("voltage", 0.0).getEntry();
  NetworkTableEntry currentEntry = tab.add("current", 0.0).getEntry();
  NetworkTableEntry encoderPositionEntry = tab.add("encoderPosition", 0.0).getEntry();
  NetworkTableEntry degreesEntry = tab.add("armDegreees", 0.0).getEntry();

  @Override
  public void robotInit() {
    // configure the arm motor
    armMotor.configFactoryDefault();
    armMotor.setInverted(TalonFXInvertType.CounterClockwise);

    tab.add("ArmMotor", armMotor);
  }

  @Override
  public void robotPeriodic() {
    if (!calibrated)
    {
      calibrateArm();
    }

    calibratedEntry.setBoolean(calibrated); 
    currentEntry.setNumber(armMotor.getStatorCurrent());
    double encoderPosition = armMotor.getSelectedSensorPosition();
    encoderPositionEntry.setNumber(encoderPosition);
    degreesEntry.setNumber(armEncoderUnitsToDegrees(encoderPosition)); 
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    // armMotor.
    
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    double joystickYAxis = -joystick.getRawAxis(1);
    double armVoltage = 0.3 * joystickYAxis;    // reduce maximum voltage while debugging (so we don't crash too hard)

    armMotor.set(ControlMode.PercentOutput, armVoltage);

    joystickEntry.setNumber(joystickYAxis);
    voltageEntry.setNumber(armVoltage);    
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

  public void calibrateArm()
  {
    // give arm a low voltage to slowly move arm upwards
    armMotor.set(ControlMode.PercentOutput, 0.2);

    // check if arm has reached the top position (current will grow very large when it stalls)
    if (armMotor.getStatorCurrent() > armMotorStallCurrentThrehold)
    {
      // stop motor where it is
      armMotor.set(ControlMode.PercentOutput, 0.0);

      // set the calibration position here
      armMotor.setSelectedSensorPosition(armDegreesToEncoderUnits(armCalibrationPositionDegrees));

      // stop future calibration
      calibrated = true;
    }
  }

  public static int armDegreesToEncoderUnits(double _degrees)
  {
    return (int)(_degrees * kEncoderUnitsPerDeg);
  }

  public static double armEncoderUnitsToDegrees(double _encoderUnits)
  {
    return (double)(_encoderUnits / kEncoderUnitsPerDeg);
  }  
}
