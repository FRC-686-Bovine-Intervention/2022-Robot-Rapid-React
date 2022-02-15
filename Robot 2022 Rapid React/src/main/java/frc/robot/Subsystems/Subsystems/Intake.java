package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Subsystems.Subsystems.Climber.ClimberState;


/**<h4>Contains all code for the Intake subsystem</h4>*/
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {if(instance == null){instance = new Intake();}return instance;}
    
    private TalonFX ArmMotor;
    private VictorSPX RollerMotor;

    private Intake()
    {
        ArmMotor = new TalonFX(Constants.kArmMotorID);
        RollerMotor = new VictorSPX(Constants.kRollerMotorID);
    
        ArmMotor.setNeutralMode(NeutralMode.Brake);
    
        intakeStatus = IntakeState.DEFENSE;
        calibrated = false;
    
        SmartDashboard.putBoolean("Intake/Enabled", true);
    }
    
    public enum ArmPosEnum {
        LOWERED(0),
        RAISED(35236);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    public ArmPosEnum targetPos;
    public double currentPos;

    public enum IntakeState {
        DEFENSE,
        INTAKE,
        OUTTAKE,
        CLIMBING,
        CALIBRATING
    }
    public enum outtakeState{
        SCORING,
        HIDING
    }

    public IntakeState intakeStatus;
    public outtakeState outtakeStatus;

    @Override
    public void run(){
        if (!calibrated){
            changeState(IntakeState.CALIBRATING);
        }
        switch(intakeStatus){
            case DEFENSE:
                setPos(ArmPosEnum.LOWERED);
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                break;
            case INTAKE:
                setPos(ArmPosEnum.LOWERED);
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0.75);
                break;
            case OUTTAKE:
                switch (outtakeStatus){
                    case SCORING:
                        setPos(ArmPosEnum.RAISED);
                        RollerMotor.set(VictorSPXControlMode.PercentOutput, -0.75);
                        break;
                    case HIDING:
                        setPos(ArmPosEnum.LOWERED);
                        RollerMotor.set(VictorSPXControlMode.PercentOutput, -0.3);
                        break;
                }
                break;
            case CLIMBING:
                //Check if robot is in defense state
                if (Climber.getInstance().ClimberStatusHistory.get(Climber.getInstance().ClimberStatusHistory.size()-1) != ClimberState.DEFENSE) break;

                //Release climber and extend
                Climber.getInstance().changeState(ClimberState.EXTEND);

                //Set ArmPosEnum.RAISED
                setPos(ArmPosEnum.RAISED);

                //If power level is high, set it back to lower state
                if (ArmMotor.getStatorCurrent() > 5) {setPos(ArmPosEnum.LOWERED);}

                //Retract climber
                Climber.getInstance().changeState(ClimberState.RETRACT);
                break;
            case CALIBRATING:
                ArmMotor.set(TalonFXControlMode.PercentOutput, 0.2);
                if (ArmMotor.getStatorCurrent() > 5)
                {
                    ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                    changeState(IntakeState.DEFENSE);
                    calibrated = true;
                }
                break;

        }
    }

    @Override
    public void runTestMode(){}

    @Override
    public void updateShuffleboard(){}

    public boolean isAtPos(ArmPosEnum pos, double error) {return ((currentPos >= pos.angleDeg - error)&&(currentPos <= pos.angleDeg + error));}
    public boolean isAtPos(ArmPosEnum pos) {return isAtPos(pos, 4);}

    public void setTargetPos(ArmPosEnum pos){}
    
    public void changeState(IntakeState newState){}
    
    private void setPos(ArmPosEnum pos){}
}