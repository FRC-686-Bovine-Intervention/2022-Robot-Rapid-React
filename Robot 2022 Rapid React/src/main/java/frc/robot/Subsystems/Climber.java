package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.controls.Controls;
import frc.robot.controls.Controls.JoystickEnum;

/**<h4>Contains all code for the Climber subsystem</h4>*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    private TalonFX LeftMotor;
    private TalonFX RightMotor;

    private Intake intake;

    private Climber()
    {
        LeftMotor = new TalonFX(Constants.kLeftClimberID);
        RightMotor = new TalonFX(Constants.kRightClimberID);

        LeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
        RightMotor.setInverted(TalonFXInvertType.Clockwise);

        RightMotor.follow(LeftMotor);

        changeState(ClimberState.DEFENSE);
        intake = Intake.getInstance();
    }

    public enum ClimberState {
        DEFENSE,
        EXTEND_FLOOR,
        RETRACT,
        EXTEND_BAR,
        CALIBRATING
    }
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();
    public boolean readyForNextState;

    @Override
    public void run()
    {
        LeftMotor.set(TalonFXControlMode.PercentOutput, Controls.getInstance().getAxis(JoystickEnum.THRUSTMASTER).y);
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

    @Override
    public void runTestMode()
    {
        run();
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

    private void setTargetPos(ClimberPos pos)
    {

    }
    
    @Override
    public void updateShuffleboard()
    {
        
    }

    public ClimberState getClimberStatus() {return ClimberStatusHistory.get(ClimberStatusHistory.size()-1);}
    public void nextState()
    {
        switch(getClimberStatus())
        {
            case DEFENSE:       changeState(ClimberState.EXTEND_FLOOR); break;
            case EXTEND_FLOOR:  changeState(ClimberState.RETRACT);      break;
            case RETRACT:       changeState(ClimberState.EXTEND_BAR);   break;
            case EXTEND_BAR:    changeState(ClimberState.RETRACT);      break;
            case CALIBRATING:   break;
        }
    }
    public void prevState() {ClimberStatusHistory.remove(ClimberStatusHistory.size()-1);}
    public void changeState(ClimberState newState) {if(getClimberStatus() != newState) ClimberStatusHistory.add(newState);}
}
