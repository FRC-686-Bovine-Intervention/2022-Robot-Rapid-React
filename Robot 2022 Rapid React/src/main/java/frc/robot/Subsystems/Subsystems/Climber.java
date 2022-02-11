package frc.robot.Subsystems.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Climber subsystem</h4>*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    private TalonFX LeftMotor;
    private TalonFX RightMotor;

    /**Has 4 states: <b>Defense</b>, <b>Extend</b>, <b>Retract</b>, <b>Lock</b><p><b>Defense</b> is when the robot is not currently climbing, and the climber is in the resting state<p><b>Extend</b> is when the climber extends and gives driver control of the pivot arms after extending when the previous state is not Defence<p><b>Retract</b> is when the climber retracts and the pivot arms move back<p><b>Lock</b> is when the pivot arms move forward to engage the bar, the climber extends a little to transfer support to the pivot arms, and the pivot arms move forward to rotate the robot*/
    public enum ClimberState {
        /**<b>Defense</b> is when the robot is not currently climbing, and the climber is in the resting state*/DEFENCE,
        /**<b>Extend</b> is when the climber extends and gives driver control of the pivot arms after extending when the previous state is not Defence*/EXTEND,
        /**<b>Retract</b> is when the climber retracts and the pivot arms move back*/RETRACT,
        /**<b>Lock</b> is when the pivot arms move forward to engage the bar, the climber extends a little to transfer support to the pivot arms, and the pivot arms move forward to rotate the robot*/LOCK
    }
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();
    /**@return <pre>{@code true} - the climber is done moving and is ready to move on
     * <p> <pre>{@code false} - the climber is still moving and should not move to the next state*/
    public boolean readyForNextState;

    public Climber()
    {
        LeftMotor = new TalonFX(Constants.kLeftClimberID);
        RightMotor = new TalonFX(Constants.kRightClimberID);

        changeState(ClimberState.DEFENCE);

        SmartDashboard.putBoolean("Climber/Enabled", true);
    }

    @Override
    public void run()
    {
        if (!SmartDashboard.getBoolean("Climber/Enabled", true))
        {
            forceChangeState(ClimberState.DEFENCE);
        }
    }

    @Override
    public void runCalibration(){}
    
    @Override
    public void updateSmartDashboard()
    {
        SmartDashboard.putString("Climber/Climber Status", getClimberStatus().name());
        SmartDashboard.putBoolean("Climber/Ready For Next State", readyForNextState);
    }
    public ClimberState getClimberStatus() {return ClimberStatusHistory.get(ClimberStatusHistory.size()-1);}
    public void nextState()
    {
        switch(getClimberStatus())
        {
            case DEFENCE:       changeState(ClimberState.EXTEND);   break;
            case EXTEND:        changeState(ClimberState.RETRACT);  break;
            case RETRACT:       changeState(ClimberState.LOCK);     break;
            case LOCK:          changeState(ClimberState.EXTEND);   break;
        }
    }
    public void prevState() {ClimberStatusHistory.remove(ClimberStatusHistory.get(ClimberStatusHistory.size()-1));}
    public void changeState(ClimberState newState) {if(SmartDashboard.getBoolean("Climber/Enabled", true)){forceChangeState(newState);};}
    private void forceChangeState(ClimberState newState) {if(getClimberStatus() != newState){ClimberStatusHistory.add(newState);}}
}
