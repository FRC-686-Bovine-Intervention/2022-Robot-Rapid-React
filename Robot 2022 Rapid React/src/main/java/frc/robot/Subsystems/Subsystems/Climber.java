package frc.robot.Subsystems.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
        LEAN_FORWARD,
        LEAN_BACKWARD,
        CALIBRATING
    } 
    public ArrayList<ClimberState> ClimberStatusHistory = new ArrayList<>();

    public enum ClimberArmPos {
        EXTEND,
        RETRACT
    }
    private ClimberArmPos currentArmPos;

    public ClimberArmPos getCurrentArmPos() {
        return currentArmPos;
    }

    public void changeCurrentArmPos(ClimberArmPos currentArmPos) {
        this.currentArmPos = currentArmPos;
    }

    public boolean isArmAtPos(ClimberArmPos state) {
        return currentArmPos == state;
    }

    @Override
    public void run(){
        if (!calibrated){
            changeState(ClimberState.CALIBRATING);
        }

        switch (getClimberStatus()){
            case DEFENSE:
                if (!isAtPos(ClimberState.LEAN_BACKWARD)){ changeState(ClimberState.LEAN_BACKWARD); }
            break;
            case LEAN_FORWARD: changeCurrentArmPos(ClimberArmPos.EXTEND); break;
            case LEAN_BACKWARD: changeCurrentArmPos(ClimberArmPos.RETRACT); break;
            case CALIBRATING:
                changeState(ClimberState.LEAN_BACKWARD);
                changeCurrentArmPos(ClimberArmPos.RETRACT);
            break;
        }

        switch (currentArmPos) {
            case EXTEND:
                if (isArmAtPos(ClimberArmPos.EXTEND)) break;
                LeftMotor.set(TalonFXControlMode.PercentOutput, 0.1);
                RightMotor.set(TalonFXControlMode.PercentOutput, 0.1);
                if (LeftMotor.getStatorCurrent() > 3 || RightMotor.getStatorCurrent() > 3) {
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0);
                    RightMotor.set(TalonFXControlMode.PercentOutput, 0);
                    changeState(ClimberState.DEFENSE);
                    calibrated = true;
                }
            break;
            case RETRACT:
                if (isArmAtPos(ClimberArmPos.RETRACT)) break;
                LeftMotor.set(TalonFXControlMode.PercentOutput, -0.1);
                RightMotor.set(TalonFXControlMode.PercentOutput, -0.1);
                if (LeftMotor.getStatorCurrent() > 3 || RightMotor.getStatorCurrent() > 3) {
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0);
                    RightMotor.set(TalonFXControlMode.PercentOutput, 0);
                    changeState(ClimberState.DEFENSE);
                    calibrated = true;
                }
            break;
        }
    }

    private boolean isAtPos(ClimberState state) {
        return getClimberStatus() == state;
    }

    @Override
    public void runTestMode(){}

    @Override
    public void updateShuffleboard(){}
    
    public ClimberState getClimberStatus() {return ClimberStatusHistory.get(ClimberStatusHistory.size()-1);}
    public void nextState()
    {
        switch(getClimberStatus())
        {
            case DEFENSE:       changeState(ClimberState.LEAN_FORWARD);   break;
            case LEAN_FORWARD:        changeState(ClimberState.LEAN_BACKWARD);  break;
            case LEAN_BACKWARD:       changeState(ClimberState.LEAN_FORWARD);   break;
            case CALIBRATING:   break;
        }
    }
    public void prevState() {ClimberStatusHistory.remove(ClimberStatusHistory.get(ClimberStatusHistory.size()-1));}
    public void changeState(ClimberState newState) {if(SmartDashboard.getBoolean("Climber/Enabled", true)){forceChangeState(newState);};}
    private void forceChangeState(ClimberState newState) {if(getClimberStatus() != newState){ClimberStatusHistory.add(newState);}}
}
