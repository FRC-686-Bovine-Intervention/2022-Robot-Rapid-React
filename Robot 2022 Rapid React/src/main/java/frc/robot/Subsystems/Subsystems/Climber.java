package frc.robot.Subsystems.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Climber subsystem</h4>
 * Has 4 states: <b>Defence</b>, <b>Extend</b>, <b>Retract</b>, and <b>Lock</b><p>
 * <b>Defence</b> is when the robot is not currently climbing, and the climber is in the resting state<p>
 * <b>Extend</b> is when the climber extends and gives driver control of the pivot arms after extending when the previous state is not Defence<p>
 * <b>Retract</b> is when the climber retracts and the pivot arms move back<p>
 * <b>Lock</b> is when the pivot arms move forward to engage the bar, the climber extends a little to transfer support to the pivot arms, and the pivot arms move forward to rotate the robot<p>
*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    public static Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    public enum ClimberState {DEFENCE, EXTEND, RETRACT, LOCK}
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();
    public boolean readyForNextState;

    public Climber()
    {
        changeState(ClimberState.DEFENCE);
    }

    @Override
    public void init() {}
    @Override
    public void run()
    {

    }
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
    public void changeState(ClimberState newState) {ClimberStatusHistory.add(newState);}
}
