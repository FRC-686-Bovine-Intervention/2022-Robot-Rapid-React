package frc.robot.subsystems; 
 
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState; 
 
/**<h4>Contains all code for the Climber subsystem</h4>*/ 
public class Climber extends Subsystem { 
    private static Climber instance = null; 
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;} 
 
    private TalonFX LeftMotor; 
    private TalonFX RightMotor; 
 
    private Intake intake; 
 
    private static final double kDefensePower = -0.07;
    private static final double kCalibratingPercent = -0.2;
    private static final double kCalibratingThreshold = 20; 
    private static final double kDisableRecalTimeThreshold = 5;

    private static final double kShaftCircum = 0.5*Math.PI*26.75/23.58; 
    private static final double kGearRatio = 5; 
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio; 
    private static final double kEncoderUnitsPerIn = kEncoderUnitsPerRev / kShaftCircum; 

    private static final double kAtTargetThresholdInches = 1;
    private static final double kIntakeMaxPercent = 0.3;
 
    private Climber() 
    { 
        LeftMotor = new TalonFX(Constants.kLeftClimberID); 
        RightMotor = new TalonFX(Constants.kRightClimberID); 
 
        
        LeftMotor.configFactoryDefault();
        LeftMotor.configOpenloopRamp(0.75);
        LeftMotor.setInverted(TalonFXInvertType.Clockwise);  
        LeftMotor.configForwardSoftLimitThreshold(inchesToEncoderUnits(ClimberPos.EXTENDED.distIn)); 
        LeftMotor.configForwardSoftLimitEnable(true); 
        LeftMotor.configReverseSoftLimitThreshold(inchesToEncoderUnits(-1.5));
        LeftMotor.configReverseSoftLimitEnable(true);
        
        RightMotor.configFactoryDefault();
        RightMotor.setInverted(TalonFXInvertType.CounterClockwise); 
        RightMotor.follow(LeftMotor); 

        calibrated = false; 
        setState(ClimberState.DEFENSE); 
        intake = Intake.getInstance(); 
    } 
 
    public enum ClimberState {  
        DEFENSE(ClimberPos.RETRACTED), 
        LOW_BAR(ClimberPos.RETRACTED),
        EXTEND_GROUND(ClimberPos.EXTENDED),
        SLOW_DRIVE(ClimberPos.EXTENDED),
        RETRACT_EXTEND(null),
        INTAKE(ClimberPos.EXTENDED),
        CALIBRATING(ClimberPos.CALIBRATION);
 
        public final ClimberPos pos; 
        ClimberState(ClimberPos pos) {this.pos = pos;} 
    }  
    private enum ClimberPos  
    {  
        EXTENDED(26.75), 
        RETRACTED(10), 
        CALIBRATION(0); 
 
        public final double distIn; 
        ClimberPos(double distIn) {this.distIn = distIn;} 
    }  
    public ClimberState climberStatus = ClimberState.DEFENSE; 
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>(); 
    public ClimberPos targetPos = ClimberPos.RETRACTED; 
    public boolean readyForNextState; 
 
    private boolean moveToClimbingMode = false;
    @Override 
    public void run() 
    { 
        disabledInit = true; 
        if(autoCalibrate && !calibrated) setState(ClimberState.CALIBRATING);
        LeftMotor.configForwardSoftLimitEnable(true);
        LeftMotor.configReverseSoftLimitEnable(true);
        // switch (climberStatus)  
        // {  
        //     case CALIBRATING:  
        //         calibrated = false;  
        //         LeftMotor.set(TalonFXControlMode.PercentOutput, kCalibratingPercent);  
        //         if (LeftMotor.getStatorCurrent() > kCalibratingThreshold)  
        //         {
        //             LeftMotor.setSelectedSensorPosition(0);
        //             calibrated = true;  
        //             prevState();  
        //             LeftMotor.set(TalonFXControlMode.PercentOutput, 0);  
        //         }  
        //     break;
        //     case DEFENSE:
        //         power = kDefensePower;
        //     default:  
        //         LeftMotor.set(TalonFXControlMode.PercentOutput, power);  
        //     break;  
        // } 

        switch(climberStatus)
        {
            case LOW_BAR:
                intake.setState(IntakeState.HARD_STOPS);
            case DEFENSE:
                LeftMotor.set(TalonFXControlMode.PercentOutput, kDefensePower);
            break;
            case EXTEND_GROUND:
                intake.setState(IntakeState.HARD_STOPS);
                LeftMotor.set(TalonFXControlMode.PercentOutput, power);
            break;
            case SLOW_DRIVE:
                intake.setState(IntakeState.MID_BAR);
                LeftMotor.set(TalonFXControlMode.PercentOutput,0);
                moveToClimbingMode = false;
            break;
            case RETRACT_EXTEND:
                if (!isAtPos(ClimberPos.RETRACTED,12) && moveToClimbingMode)
                {
                    intake.setState(IntakeState.CLIMBING);
                }
                else
                {
                    moveToClimbingMode = false;
                    intake.setClimbingPower(0);
                    if (intake.intakeStatus == IntakeState.MID_BAR && !isAtPos(ClimberPos.RETRACTED,12))
                    {
                        intake.setState(IntakeState.MID_BAR);
                    }
                    else
                    {
                        intake.setState(IntakeState.HARD_STOPS);
                    }
                }
                LeftMotor.set(TalonFXControlMode.PercentOutput, power);
            break;
            case INTAKE:
                moveToClimbingMode = true;
                intake.setState(IntakeState.CLIMBING);
                intake.setClimbingPower(power * kIntakeMaxPercent);
                LeftMotor.set(TalonFXControlMode.PercentOutput,0);
            break;
            case CALIBRATING:  
                calibrated = false;  
                LeftMotor.set(TalonFXControlMode.PercentOutput, kCalibratingPercent);
                LeftMotor.configReverseSoftLimitEnable(false);
                if (LeftMotor.getStatorCurrent() > kCalibratingThreshold)  
                {
                    LeftMotor.setSelectedSensorPosition(inchesToEncoderUnits(ClimberPos.CALIBRATION.distIn));
                    resetState();
                    calibrated = true;
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0);  
                }  
            break;
        }
        power = 0;
    } 
 
    private boolean disabledInit = true; 
    private double disabledTime; 
    @Override 
    public void disable() { 
        if(disabledInit) disabledTime = Timer.getFPGATimestamp(); 
        if(Timer.getFPGATimestamp() - disabledTime > kDisableRecalTimeThreshold) calibrated = false; 
        disabledInit = false; 
    } 
 
    @Override 
    public void runTestMode() 
    { 
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
    public void runCalibration() { 
        calibrated = false; 
        setState(ClimberState.CALIBRATING); 
    } 
 
    public boolean isAtPos(ClimberPos pos, double threshold) 
    { 
        double currentDistanceInches = encoderUnitsToInches(LeftMotor.getSelectedSensorPosition()); 
        double targetInches = pos.distIn; 
 
        return (Math.abs(currentDistanceInches - targetInches) < threshold); 
    } 
 
    public boolean isAtPos(ClimberPos pos) {return isAtPos(pos,kAtTargetThresholdInches);} 
 
//DEBUxG  
private double power;  
    public void setTargetPos(double power)  
    {  
        this.power = power;  
    }
    
    private static int inchesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerIn);} 
    private static double encoderUnitsToInches(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerIn);} 
  
    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");  
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();  
    private NetworkTableEntry calibrateButton = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();  
    private NetworkTableEntry statusEntry = tab.add("Status", "not updating").withWidget(BuiltInWidgets.kTextView).getEntry();  
    private NetworkTableEntry historyEntry = tab.add("Status History", "not updating").withWidget(BuiltInWidgets.kTextView).getEntry();  
    private NetworkTableEntry enableEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry(); 
    private NetworkTableEntry climberCurrentPosEntry = tab.add("Current Pos", -9999).withWidget(BuiltInWidgets.kTextView)       .withPosition(9,0).getEntry(); 

    private NetworkTableEntry lowbarEntry           = tab.add("Low Bar", false)         .withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 3).getEntry();
    private NetworkTableEntry extendGroundEntry     = tab.add("Extend Ground", false)   .withWidget(BuiltInWidgets.kBooleanBox).withPosition(1, 3).getEntry();
    private NetworkTableEntry slowDriveEntry        = tab.add("Slow Drive", false)      .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 3).getEntry();
    private NetworkTableEntry retractExtendEntry    = tab.add("Retract|Extend", false)  .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 3).getEntry();
    private NetworkTableEntry intakeEntry           = tab.add("Intake", false)          .withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 3).getEntry();
      
    @Override  
    public void updateShuffleboard()  
    {
        statusEntry.setString(climberStatus.name()); 
        climberCurrentPosEntry.setNumber(encoderUnitsToInches(LeftMotor.getSelectedSensorPosition())); 
        calibratedEntry.setBoolean(calibrated);  
        Enabled = enableEntry.getBoolean(true);  

        historyEntry.setString(ClimberStatusHistory.toString());
        
        lowbarEntry.setBoolean(false);
        extendGroundEntry.setBoolean(false);
        slowDriveEntry.setBoolean(false);
        retractExtendEntry.setBoolean(false);
        intakeEntry.setBoolean(false);
        switch(climberStatus)
        {
            case INTAKE:
                intakeEntry.setBoolean(true);
            case RETRACT_EXTEND:
                retractExtendEntry.setBoolean(true);
            case SLOW_DRIVE:
                slowDriveEntry.setBoolean(true);
            case EXTEND_GROUND:
                extendGroundEntry.setBoolean(true);
            case LOW_BAR:
                lowbarEntry.setBoolean(true);
            default: break;
        }
    }  
 
    public ClimberState getClimberStatus() { 
        try 
        { 
            return ClimberStatusHistory.get(ClimberStatusHistory.size()-1); 
        } 
        catch (IndexOutOfBoundsException o) 
        { 
            return null; 
        } 
    } 
    public void nextState() 
    { 
        switch(climberStatus) 
        { 
            case DEFENSE:           setState(ClimberState.LOW_BAR);         break;
            case LOW_BAR:           setState(ClimberState.EXTEND_GROUND);   break;
            case EXTEND_GROUND:     setState(ClimberState.SLOW_DRIVE);      break;
            case SLOW_DRIVE:        setState(ClimberState.RETRACT_EXTEND);  break;
            case RETRACT_EXTEND:    setState(ClimberState.INTAKE);          break;
            case INTAKE:            setState(ClimberState.RETRACT_EXTEND);  break;
            case CALIBRATING:       break; 
        } 
    } 
    public void prevState()  
    {  
        try  
        {  
            ClimberStatusHistory.remove(ClimberStatusHistory.size()-1);  
            climberStatus = ClimberStatusHistory.get(ClimberStatusHistory.size()-1);  
        }  
        catch (IndexOutOfBoundsException exception)  
        {  
            resetState();
        }  
    } 

    public void resetState()
    {
        ClimberStatusHistory.clear();
        ClimberStatusHistory.add(ClimberState.DEFENSE);  
        climberStatus = ClimberState.DEFENSE;  
    }
      
    public void setState(ClimberState newState)  
    {  
        if (climberStatus != newState && climberStatus != ClimberState.CALIBRATING)  
        {  
            ClimberStatusHistory.add(newState);  
            climberStatus = newState;  
        }  
    }  
} 
