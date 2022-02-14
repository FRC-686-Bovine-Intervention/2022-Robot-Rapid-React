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
    
    private Climber()
    {
        LeftMotor = new TalonFX(Constants.kLeftClimberID);
        RightMotor = new TalonFX(Constants.kRightClimberID);

        changeState(ClimberState.DEFENSE);
        calibrated = false;

        SmartDashboard.putBoolean("Climber/Enabled", true);
    }

    public enum ClimberState {
        DEFENSE,
        EXTEND,
        RETRACT,
        CALIBRATING
    }
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();

    @Override
    public void run(){}

    @Override
    public void runTestMode(){}

    @Override
    public void updateShuffleboard(){}
    
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
    public void changeState(ClimberState newState) {if(SmartDashboard.getBoolean("Climber/Enabled", true)){forceChangeState(newState);};}
    private void forceChangeState(ClimberState newState) {if(getClimberStatus() != newState){ClimberStatusHistory.add(newState);}}
}
