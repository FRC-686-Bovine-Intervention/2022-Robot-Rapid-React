package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Intake subsystem</h4>*/
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {if(instance == null){instance = new Intake();}return instance;}
    
    private TalonFX ArmMotor;
    private VictorSPX RollerMotor;
    
    private static final double kOuttakePercentOutput = -0.9;
    private static final double kIntakePercentOutput = 0.7;

    TrapezoidProfile.State calState = new TrapezoidProfile.State(ArmPosEnum.CALIBRATION.angleDeg, 0);
    TrapezoidProfile.State scoringState = new TrapezoidProfile.State(ArmPosEnum.RAISED.angleDeg, 0);
    TrapezoidProfile.State groundState = new TrapezoidProfile.State(ArmPosEnum.LOWERED.angleDeg, 0);

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

    private ProfiledPIDController pid;

    private Intake()
    {
        ArmMotor = new TalonFX(Constants.kArmMotorID);
        RollerMotor = new VictorSPX(Constants.kRollerMotorID);
    
        ArmMotor.configFactoryDefault();
        ArmMotor.setInverted(TalonFXInvertType.CounterClockwise);
        ArmMotor.setNeutralMode(NeutralMode.Brake);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocityDegPerSecond, kMaxAccelerationDegPerSecSquared);
        pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.reset(calState);
    
        calibrated = false;
    }
    
    public enum ArmPosEnum {
        LOWERED(0),
        RAISED(105),
        CALIBRATION(112);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    public ArmPosEnum targetPos = ArmPosEnum.RAISED;
    public double currentPos;

    public enum IntakeState {
        DEFENSE,
        INTAKE,
        OUTTAKE,
        CLIMBING,
        CALIBRATING
    }
    public IntakeState intakeStatus = IntakeState.DEFENSE;

    @Override
    public void run()
    {
        if(autoCalibrate && !calibrated) {changeState(IntakeState.CALIBRATING);}
        switch (intakeStatus)
        {
            case DEFENSE: default:
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                setTargetPos(ArmPosEnum.RAISED);
            break;
            case INTAKE:
                if(isAtPos(ArmPosEnum.LOWERED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kIntakePercentOutput);}
                else {setTargetPos(ArmPosEnum.LOWERED);}
            break;
            case OUTTAKE:
                if(isAtPos(ArmPosEnum.RAISED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kOuttakePercentOutput);}
                else {setTargetPos(ArmPosEnum.RAISED);}
            break;
            case CLIMBING: break;
            case CALIBRATING:
                if (checkFwdLimitSwitch())
                {
                    ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                    changeState(IntakeState.DEFENSE);
                    calibrated = true;
                    break;
                }
                ArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
            break;
        }
        if (intakeStatus != IntakeState.CALIBRATING) setPos(targetPos);
    }

    @Override
    public void runTestMode()
    {
        //intakeStatus = stateChooser.getSelected();
        if (stateChooser.getSelected() == IntakeState.CALIBRATING || calibrateEntry.getBoolean(false))
        {
            runCalibration();
            calibrateEntry.setBoolean(false);
        }
        autoCalibrate = false;
        run();
        autoCalibrate = true;
    }
    @Override
    public void runCalibration()
    {
        calibrated = false;
        changeState(IntakeState.CALIBRATING);
    }
    @Override
    public void disable() {
        ArmMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        calibrated = false;
    }

    /**
     * @param pos is the desired pos
     * @param error is the radius of error
     * @return if the arm is within the radius of error from the desired pos
     */
    public boolean isAtPos(ArmPosEnum pos)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition());
        double targetDegrees = pos.angleDeg;

        return (Math.abs(currentAngleDegrees - targetDegrees) < kAtTargetThresholdDegrees);
    }

    public void setTargetPos(ArmPosEnum pos) {targetPos = pos;}

    private void setPos(ArmPosEnum pos)
    {
        pid.setGoal(pos.angleDeg);
        double pidOutput = 0.0;
        double currentAngleDegrees = encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition());

        pidOutput = pid.calculate(currentAngleDegrees);
        ArmMotor.set(TalonFXControlMode.PercentOutput, pidOutput);

        if ((pid.getGoal().position == ArmPosEnum.LOWERED.angleDeg) && (currentAngleDegrees < kGroundHoldingThresholdDegrees))
        {
            ArmMotor.set(TalonFXControlMode.PercentOutput, kGroundHoldingPercentOutput);
        }
    }

    public static int degreesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerDeg);}
    public static double encoderUnitsToDegrees(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerDeg);}

    public boolean checkFwdLimitSwitch()
    {
        boolean fwdLimitSwitchClosed = (ArmMotor.isFwdLimitSwitchClosed() == 1);
        if (fwdLimitSwitchClosed)
        {
            // set the calibration position here
            ArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(ArmPosEnum.CALIBRATION.angleDeg));
            pid.reset(calState);
            pid.setGoal(calState);

            // stop future calibration

        }
        return fwdLimitSwitchClosed;
    }

    public void changeState(IntakeState newState)
    {
        intakeStatus = newState;
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private NetworkTableEntry calibrateEntry = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry enableEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private NetworkTableEntry statusEntry = tab.add("Status", "not updating what").withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry armposEntry = tab.add("Arm position", "not updating what").withWidget(BuiltInWidgets.kTextView).getEntry();
    private SendableChooser<IntakeState> stateChooser = new SendableChooser<>();

    @Override
    public void updateShuffleboard()
    {
        Enabled = enableEntry.getBoolean(true);
        enableEntry.setBoolean(Enabled);
        statusEntry.setString(intakeStatus.name());
        armposEntry.setString(targetPos.name());
    }
}