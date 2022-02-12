package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Intake subsystem</h4>*/
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {if(instance == null){instance = new Intake();}return instance;}
    
    private TalonFX ArmMotor;
    private VictorSPX RollerMotor;

    public Intake()
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
        RAISED(35236),
        CLIMBERRAISE(1000);

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
    public IntakeState intakeStatus;

    @Override
    public void run()
    {
        if(!calibrated) {changeState(IntakeState.CALIBRATING);}
        //if(!SmartDashboard.getBoolean("Intake/Enabled", true)){intakeStatus = IntakeState.DEFENSE;}
        switch (intakeStatus)
        {
            case DEFENSE: default:
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                setTargetPos(ArmPosEnum.RAISED);
            break;
            case INTAKE:
                if (isAtPos(ArmPosEnum.LOWERED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, 0.3);}
                else {setTargetPos(ArmPosEnum.LOWERED);}
            break;
            case OUTTAKE:
                if (isAtPos(ArmPosEnum.RAISED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, -0.3);}
                else {setTargetPos(ArmPosEnum.RAISED);}
            break;
            case CLIMBING: break;
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
        setPos(targetPos);
    }

    /**Is true if the robot has calibrated in the past, otherwise false */
    private boolean calibrated;

    @Override
    public void runCalibration() {calibrated = false;}

    /**
     * @param pos is the desired pos
     * @param error is the radius of error
     * @return if the arm is within the radius of error from the desired pos
     */
    public boolean isAtPos(ArmPosEnum pos, double error) {return ((currentPos >= pos.angleDeg - error)&&(currentPos <= pos.angleDeg + error));}
    public boolean isAtPos(ArmPosEnum pos) {return isAtPos(pos, 4);}

    public void setTargetPos(ArmPosEnum pos) {targetPos = pos;}

    private void setPos(ArmPosEnum pos)
    {
        
    }

    @Override
    public void updateSmartDashboard()
    {
        SmartDashboard.putString("Intake/Status", intakeStatus.name());
        SmartDashboard.putNumber("Intake/ArmPos", currentPos);
        SmartDashboard.putString("Intake/TargetPos", targetPos.name());
    }
    public void changeState(IntakeState newState)
    {
        intakeStatus = newState;
    }
}