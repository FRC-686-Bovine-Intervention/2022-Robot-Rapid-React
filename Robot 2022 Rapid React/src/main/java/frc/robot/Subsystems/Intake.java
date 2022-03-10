package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

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

    private static final double kCalibrationPercentOutput = 0.2;

    private static final double kGroundHoldingThresholdDegrees = 2.0;
    private static final double kGroundHoldingPercentOutput = -0.10;

    private static final double kGearRatio = 16.0 * 48.0/12.0;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;

    private static final double kP = 0.06;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 240;
    private static final double kMaxAccelerationDegPerSecSquared = 390;

    private static final double kAtTargetThresholdDegrees = 5.0;
    public static final double kClimbingHoldPercent = 0.2;

    private static final double kDisableRecalTimeThreshold = 5;

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

        for (IntakeState s : IntakeState.values())
        {
            stateChooser.addOption(s.name(), s);
        }
    
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
        DEFENSE(ArmPosEnum.RAISED),
        INTAKE(ArmPosEnum.LOWERED),
        OUTTAKE(ArmPosEnum.RAISED),
        OUTTAKE_GROUND(ArmPosEnum.LOWERED),
        CLIMBING(null),
        CALIBRATING(null);

        public final ArmPosEnum armPos;
        IntakeState(ArmPosEnum armPos) {this.armPos = armPos;}
    }
    public IntakeState intakeStatus = IntakeState.DEFENSE;

    @Override
    public void run()
    {
        disabledInit = true;
        if(autoCalibrate && !calibrated) {setState(IntakeState.CALIBRATING);}
        switch (intakeStatus)
        {
            case DEFENSE: default:
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                setTargetPos(ArmPosEnum.RAISED);
                setPos(targetPos);
            break;
            case INTAKE:
                if(isAtPos(ArmPosEnum.LOWERED, 30)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kIntakePercentOutput);}
                else {setTargetPos(ArmPosEnum.LOWERED);}
                setPos(targetPos);
            break;
            case OUTTAKE:
                if(isAtPos(ArmPosEnum.RAISED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kOuttakePercentOutput);}
                else {setTargetPos(ArmPosEnum.RAISED);}
                setPos(targetPos);
            break;
            case OUTTAKE_GROUND:
                if(isAtPos(ArmPosEnum.LOWERED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kOuttakePercentOutput);}
                else {setTargetPos(ArmPosEnum.LOWERED);}
                setPos(targetPos);
            break;
            case CLIMBING:
                ArmMotor.set(TalonFXControlMode.PercentOutput, climbingPower);
            break;
            case CALIBRATING:
                if (checkFwdLimitSwitch())
                {
                    ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                    setState(IntakeState.DEFENSE);
                    calibrated = true;
                    break;
                }
                calibrated = false;
                ArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
                break;
            }
        climbingPower = 0;
    }

    @Override
    public void runTestMode()
    {
        if (stateChooser.getSelected() != null) intakeStatus = stateChooser.getSelected();
        if (calibrateButton.getBoolean(false))
        {
            calibrateButton.setBoolean(false);
            runCalibration();
        }
        autoCalibrate = false;
        run();
        autoCalibrate = true;
    }
    @Override
    public void runCalibration()
    {
        calibrated = false;
        setState(IntakeState.CALIBRATING);
    }
    private boolean disabledInit = true;
    private double disabledTime;
    @Override
    public void disable() {
        ArmMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        RollerMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        if(disabledInit) disabledTime = Timer.getFPGATimestamp();
        if(Timer.getFPGATimestamp() - disabledTime > kDisableRecalTimeThreshold) calibrated = false;
        disabledInit = false;
    }

    /**
     * @param pos is the desired pos
     * @param error is the radius of error
     * @return if the arm is within the radius of error from the desired pos
     */
    public boolean isAtPos(ArmPosEnum pos, double threshold)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition());
        double targetDegrees = pos.angleDeg;

        return (Math.abs(currentAngleDegrees - targetDegrees) < threshold);
    }

    public boolean isAtPos(ArmPosEnum pos) {return isAtPos(pos,kAtTargetThresholdDegrees);}

    public void setTargetPos(ArmPosEnum pos) {targetPos = pos;}

    private double climbingPower;
    public void setClimbingPower(double armPower)
    {
        climbingPower = armPower;
    }

    double pidOutput = 0.0;
    private void setPos(ArmPosEnum pos)
    {
        pid.setGoal(pos.angleDeg);
        pidOutput = 0.0;
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

    public void setState(IntakeState newState)
    {
        intakeStatus = newState;
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private NetworkTableEntry statusEntry = tab.add("Status", "not updating what").withWidget(BuiltInWidgets.kTextView)         .withPosition(0,0).withSize(2,1).getEntry();
    private NetworkTableEntry armposEntry = tab.add("Arm position", "not updating what").withWidget(BuiltInWidgets.kTextView)   .withPosition(0,1).getEntry();
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(1,1).getEntry();
    private NetworkTableEntry calibrateButton = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton)            .withPosition(1,3).getEntry();
    private NetworkTableEntry enableEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch)                    .withPosition(0,3).getEntry();
    private SendableChooser<IntakeState> stateChooser = new SendableChooser<>();
    private ComplexWidget wig = tab.add("State Chooser", stateChooser)                                                          .withPosition(0,4).withSize(2,1);
    private NetworkTableEntry armCurrentEntry = tab.add("Arm Current", -9999).withWidget(BuiltInWidgets.kTextView)              .withPosition(8,0).getEntry();
    private NetworkTableEntry armCurrentPosEntry = tab.add("Arm Current Pos", -9999).withWidget(BuiltInWidgets.kTextView)       .withPosition(9,0).getEntry();
    private NetworkTableEntry armPIDOutputEntry = tab.add("Arm PID Output", -9999).withWidget(BuiltInWidgets.kTextView)         .withPosition(8,1).getEntry();
    private NetworkTableEntry armGoalEntry = tab.add("Arm PID Goal", -9999).withWidget(BuiltInWidgets.kTextView)                .withPosition(9,1).getEntry();

    @Override
    public void updateShuffleboard()
    {
        armCurrentEntry.setDouble(ArmMotor.getStatorCurrent());
        armPIDOutputEntry.setDouble(pidOutput);
        armCurrentPosEntry.setDouble(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()));
        armGoalEntry.setDouble(pid.getGoal().position);
        Enabled = enableEntry.getBoolean(true);
        enableEntry.setBoolean(Enabled);
        statusEntry.setString(intakeStatus.name());
        armposEntry.setString(targetPos.name());
        calibratedEntry.setBoolean(calibrated);
    }
}