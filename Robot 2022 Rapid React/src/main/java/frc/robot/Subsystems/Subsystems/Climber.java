package frc.robot.Subsystems.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Subsystems.Subsystems.Intake.ArmPosEnum;
import frc.robot.Subsystems.Subsystems.Intake.IntakeState;

/**<h4>Contains all code for the Climber subsystem</h4>*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    private TalonFX LeftMotor;
    private TalonFX RightMotor;

    public enum ClimberState {
        DEFENSE,
        EXTEND,
        RETRACT,
        CALIBRATING
    }
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();
    /**@return <pre>{@code true} - the climber is done moving and is ready to move on
     * <p> <pre>{@code false} - the climber is still moving and should not move to the next state*/
    public boolean readyForNextState;

    private Climber()
    {
        LeftMotor = new TalonFX(Constants.kLeftClimberID);
        RightMotor = new TalonFX(Constants.kRightClimberID);

        changeState(ClimberState.DEFENSE);
    }

    @Override
    public void run()
    {
        if(!calibrated && !DriverStation.isTest()) {changeState(ClimberState.CALIBRATING);}
        if ((getClimberStatus() != ClimberState.DEFENSE) && (getClimberStatus() != ClimberState.CALIBRATING)) Intake.getInstance().changeState(IntakeState.CLIMBING);
        switch (getClimberStatus())
        {
            case DEFENSE:
                setTargetPos(ClimberPos.RETRACTED);
            break;
            case EXTEND:
                if (ClimberStatusHistory.get(ClimberStatusHistory.size()-2) == ClimberState.DEFENSE)
                {
                    if (Intake.getInstance().isAtPos(ArmPosEnum.RAISED))
                    {
                        setTargetPos(ClimberPos.EXTENDED);
                    }
                    else
                    {
                        Intake.getInstance().setTargetPos(ArmPosEnum.RAISED);
                    }
                }
                else
                {
                    if (isAtPos(ClimberPos.EXTENDED))
                    {
                        Intake.getInstance(); //Driver control
                    }
                    else
                    {
                        setTargetPos(ClimberPos.EXTENDED);
                    }
                }
            break;
            case RETRACT:
                if (isAtPos(ClimberPos.RETRACTED))
                {
                    Intake.getInstance().setTargetPos(ArmPosEnum.LOWERED);
                }
                else
                {
                    setTargetPos(ClimberPos.RETRACTED);
                }
            break;
            case CALIBRATING:

            break;
        }
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
        SmartDashboard.putString("Climber/Climber Status", getClimberStatus().name());
        SmartDashboard.putBoolean("Climber/Ready For Next State", readyForNextState);
    }
    public ClimberState getClimberStatus() {return ClimberStatusHistory.get(ClimberStatusHistory.size()-1);}
    public void nextState()
    {
        switch(getClimberStatus())
        {
            case DEFENSE:       changeState(ClimberState.EXTEND);   break;
            case EXTEND:        changeState(ClimberState.RETRACT);  break;
            case RETRACT:       changeState(ClimberState.EXTEND);   break;
            case CALIBRATING:   break;
        }
    }
    public void prevState() {ClimberStatusHistory.remove(ClimberStatusHistory.get(ClimberStatusHistory.size()-1));}
    public void changeState(ClimberState newState) {if(getClimberStatus() != newState) ClimberStatusHistory.add(newState);}
}
