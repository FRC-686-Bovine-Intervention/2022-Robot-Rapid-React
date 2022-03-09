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

/**<h4>Contains all code for the Climber subsystem</h4>*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    private TalonFX LeftMotor;
    private TalonFX RightMotor;

    private Intake intake;

    private static final double kCalibratingPercent = -0.15; 
    private static final double kCalibratingThreshold = 20;
    private static final double kDisableRecalTimeThreshold = 5;

    private Climber()
    {
        LeftMotor = new TalonFX(Constants.kLeftClimberID);
        RightMotor = new TalonFX(Constants.kRightClimberID);

        LeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
        RightMotor.setInverted(TalonFXInvertType.Clockwise);

        RightMotor.follow(LeftMotor);

        calibrated = false;
        setState(ClimberState.DEFENSE);
        intake = Intake.getInstance();
    }

    public enum ClimberState {
        DEFENSE,
        EXTEND_FLOOR,
        RETRACT,
        EXTEND_BAR,
        CALIBRATING
    }
    public ClimberState climberStatus = ClimberState.DEFENSE; 
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();
    public boolean readyForNextState;

    @Override
    public void run()
    {
        disabledInit = true;
        if(autoCalibrate && !calibrated) setState(ClimberState.CALIBRATING); 
        switch (climberStatus) 
        { 
            case CALIBRATING: 
                calibrated = false; 
                LeftMotor.set(TalonFXControlMode.PercentOutput, kCalibratingPercent); 
                if (LeftMotor.getStatorCurrent() > kCalibratingThreshold) 
                { 
                    calibrated = true; 
                    prevState(); 
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0); 
                } 
            break; 
            default: 
                LeftMotor.set(TalonFXControlMode.PercentOutput, power); 
            break; 
        } 
        power = 0; 
        // if(autoCalibrate && !calibrated) {changeState(ClimberState.CALIBRATING);}
        // switch (getClimberStatus())
        // {
        //     case DEFENSE:
        //         setTargetPos(ClimberPos.RETRACTED);
        //     break;
        //     case EXTEND_FLOOR:
        //         intake.changeState(IntakeState.CLIMBING);
        //         if (intake.isAtPos(ArmPosEnum.RAISED))
        //         {
        //             setTargetPos(ClimberPos.EXTENDED);
        //         }
        //         else
        //         {
        //             intake.setTargetPos(ArmPosEnum.RAISED);
        //         }
        //     break;
        //     case RETRACT:
        //         intake.changeState(IntakeState.CLIMBING);
        //         if (isAtPos(ClimberPos.RETRACTED))
        //         {
        //             intake.setTargetPos(ArmPosEnum.LOWERED);
        //         }
        //         else
        //         {
        //             setTargetPos(ClimberPos.RETRACTED);
        //         }
        //     break;
        //     case EXTEND_BAR:
        //         intake.changeState(IntakeState.CLIMBING);
        //         if (isAtPos(ClimberPos.EXTENDED))
        //         {
        //             //Driver control
        //         }
        //         else
        //         {
        //             setTargetPos(ClimberPos.EXTENDED);
        //         }
        //     break;
        //     case CALIBRATING:

        //     break;
        // }
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

    private boolean isAtPos(ClimberPos pos)
    {
        return true;
    }

    private enum ClimberPos
    {
        EXTENDED,
        RETRACTED
    }
//DEBUG 
private double power; 
    public void setTargetPos(double power) 
    { 
        this.power = power; 
    } 

    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    private NetworkTableEntry calibrateButton = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private NetworkTableEntry enableEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    
    @Override
    public void updateShuffleboard()
    {
        calibratedEntry.setBoolean(calibrated);
        Enabled = enableEntry.getBoolean(true);
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
        switch(getClimberStatus())
        {
            case DEFENSE:       setState(ClimberState.EXTEND_FLOOR); break;
            case EXTEND_FLOOR:  setState(ClimberState.RETRACT);      break;
            case RETRACT:       setState(ClimberState.EXTEND_BAR);   break;
            case EXTEND_BAR:    setState(ClimberState.RETRACT);      break;
            case CALIBRATING:   break;
        }
    }
    public void prevState() 
    { 
        try 
        { 
            climberStatus = ClimberStatusHistory.get(ClimberStatusHistory.size()-1); 
            ClimberStatusHistory.remove(ClimberStatusHistory.size()-1); 
        } 
        catch (IndexOutOfBoundsException exception) 
        { 
            climberStatus = ClimberState.DEFENSE; 
        } 
    } 
     
    public void setState(ClimberState newState) 
    { 
        if (climberStatus != newState) 
        { 
            ClimberStatusHistory.add(newState); 
            climberStatus = newState; 
        } 
    } 
}
