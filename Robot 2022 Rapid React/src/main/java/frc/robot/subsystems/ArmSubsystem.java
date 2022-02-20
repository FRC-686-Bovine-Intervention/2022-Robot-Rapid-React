package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.lib.util.Util;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends Subsystem
{
    public static final double kCalibrationPositionDegrees = 112;
    public static final double kScoringPositionDegrees = 90.0;
    public static final double kGroundPositionDegrees = 0.0;    
    
    TrapezoidProfile.State calState = new TrapezoidProfile.State(kCalibrationPositionDegrees, 0);
    TrapezoidProfile.State scoringState = new TrapezoidProfile.State(kScoringPositionDegrees, 0);
    TrapezoidProfile.State groundState = new TrapezoidProfile.State(kGroundPositionDegrees, 0);

    private static final double kCalibrationPercentOutput = 0.15;

    private static final double kGroundHoldingThresholdDegrees = 2.0;
    private static final double kGroundHoldingPercentOutput = -0.10;

    private static final double kGearRatio = 16.0 * 48.0/12.0;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;

    private static final double kP = 0.08;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 120;
    private static final double kMaxAccelerationDegPerSecSquared = 270;

    private static final double kAtTargetThresholdDegrees = 1.0;

    public WPI_TalonFX motor = new WPI_TalonFX(Constants.kArmMotorID);
    private ProfiledPIDController pid;

    public boolean calibrated = false;

    ShuffleboardTab tab = Shuffleboard.getTab("ArmDebug");
    NetworkTableEntry calibratedEntry = tab.add("calibrated", false).getEntry();
    NetworkTableEntry joystickEntry = tab.add("joystick", 0.0).getEntry();
    NetworkTableEntry voltageEntry = tab.add("voltage", 0.0).getEntry();
    NetworkTableEntry currentEntry = tab.add("current", 0.0).getEntry();
    NetworkTableEntry encoderPositionEntry = tab.add("encoderPosition", 0.0).getEntry();
    NetworkTableEntry degreesEntry = tab.add("armDegrees", 0.0).getEntry();     
    NetworkTableEntry goalEntry = tab.add("pidGoal", 0.0).getEntry();   
    NetworkTableEntry setPtEntry = tab.add("pidSetPt", 0.0).getEntry();  
    NetworkTableEntry pidOutEntry = tab.add("pidOut", 0.0).getEntry(); 
    NetworkTableEntry limitSwitchEntry = tab.add("fwdLimitSwitch", false).getEntry();
    NetworkTableEntry manualModeEntry = tab.add("manualMode", false).getEntry();
    
    public ArmSubsystem() 
    {
        // configure the arm motor
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
                
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocityDegPerSecond, kMaxAccelerationDegPerSecSquared);
        pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.reset(calState);

        calibrated = false;         
    }


    public void setTarget(double _targetDegrees)
    {
        System.out.printf("setTarget to %.1f degrees\n", _targetDegrees);
        pid.setGoal(_targetDegrees);
    }

    public boolean isAtTarget(double _targetDegrees)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(motor.getSelectedSensorPosition());
        double targetDegrees = pid.getGoal().position;

        return (abs(currentAngleDegrees - targetDegrees) < kAtTargetThresholdDegrees);
    }

    public void onLoop()
    {
        double pidOutput = 0.0;
        double currentAngleDegrees = encoderUnitsToDegrees(motor.getSelectedSensorPosition());

        if (calibrated)
        {
            pidOutput = pid.calculate(currentAngleDegrees);
            // pidOutput = Util.limit(pidOutput, 0.4);
            motor.set(ControlMode.PercentOutput, pidOutput);

            if ((pid.getGoal().position == kGroundPositionDegrees) && (currentAngleDegrees < kGroundHoldingThresholdDegrees))
            {
                motor.set(ControlMode.PercentOutput, kGroundHoldingPercentOutput);
            }
        }



        calibratedEntry.setBoolean(calibrated); 
        currentEntry.setNumber(motor.getStatorCurrent());
        degreesEntry.setNumber(currentAngleDegrees); 
        goalEntry.setNumber(pid.getGoal().position);
        setPtEntry.setNumber(pid.getSetpoint().position); 
        pidOutEntry.setNumber(pidOutput); 
        limitSwitchEntry.setBoolean((motor.isFwdLimitSwitchClosed() == 1)); 
    }

    public void stop()
    {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    public void zeroSensors()
    {
        // give arm a low voltage to slowly move arm upwards
        motor.set(ControlMode.PercentOutput, kCalibrationPercentOutput);
    
        // check if arm has reached the top position (current will grow very large when it stalls)
        if (checkFwdLimitSwitch())
        {
            calibrated = true;                
            System.out.printf("Arm Calibration Complete!\n");
        }
    }

    public static int degreesToEncoderUnits(double _degrees)
    {
      return (int)(_degrees * kEncoderUnitsPerDeg);
    }
  
    public static double encoderUnitsToDegrees(double _encoderUnits)
    {
      return (double)(_encoderUnits / kEncoderUnitsPerDeg);
    }   


    public boolean checkFwdLimitSwitch()
    {
        boolean fwdLimitSwitchClosed = (motor.isFwdLimitSwitchClosed() == 1);
        if (fwdLimitSwitchClosed)
        {
            // set the calibration position here
            motor.setSelectedSensorPosition(degreesToEncoderUnits(kCalibrationPositionDegrees));
            pid.reset(calState);
            pid.setGoal(calState);

            // stop future calibration

        }
        return fwdLimitSwitchClosed;
    }
   
    public void manualControl(double _ctrl)
    {
        joystickEntry.setNumber(_ctrl);
        motor.set(ControlMode.PercentOutput, _ctrl);
    }
  
}
  
