package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    private static double RaisedPos = 35236;
    private static double LoweredPos = 0;

    /**Has 4 states: <b>Defence</b>, <b>Intake</b>, <b>Outtake</b>, <b>Climbing</b><p><b>Defence</b> is when the intake is resting and the rollers are not spinning<p><b>Intake</b> is when the intake is down and the rollers are rolling balls in<p><b>Outtake</b> is when the intake is up and the rollers are rolling balls out<p><b>Climbing</b> is when the intake is being controlled by the climber*/
    public enum IntakeState {
        /**<b>Defence</b> is when the intake is resting and the rollers are not spinning*/DEFENCE,
        /**<b>Intake</b> is when the intake is down and the rollers are rolling balls in*/INTAKE,
        /**<b>Outtake</b> is when the intake is up and the rollers are rolling balls out*/OUTTAKE,
        /**<b>Climbing</b> is when the intake is being controlled by the climber*/CLIMBING,
        CALIBRATING
    }
    public IntakeState intakeStatus;

    public Intake()
    {
        ArmMotor = new TalonFX(Constants.kArmMotorID);
        RollerMotor = new VictorSPX(Constants.kRollerMotorID);

        ArmMotor.setNeutralMode(NeutralMode.Brake);

        intakeStatus = IntakeState.DEFENCE;

        SmartDashboard.putBoolean("Intake/Enabled", true);
    }

    @Override
    public void run()
    {
        //if(!SmartDashboard.getBoolean("Intake/Enabled", true)){intakeStatus = IntakeState.DEFENCE;}
        switch (intakeStatus)
        {
            case DEFENCE: default:
            System.out.println("intake in defence state");
                RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                //ArmMotor.setPos();
            break;
            case INTAKE:
            System.out.println("intake in intake state");
            /*
                if (WithinRange(ArmMotor.getSelectedSensorPosition(),LoweredPos,1000))
                {
                    RollerMotor.set(VictorSPXControlMode.PercentOutput, 0.3);
                }
                else
                {
                    //ArmMotor.setPos();
                }
            */
            break;
            case OUTTAKE:
            System.out.println("intake in outtake state");
            /*
                if (WithinRange(ArmMotor.getSelectedSensorPosition(),RaisedPos,1000))
                {
                    RollerMotor.set(VictorSPXControlMode.PercentOutput, -0.3);
                }
                else
                {
                    //ArmMotor.setPos();
                }
            */
            break;
            case CALIBRATING:
                ArmMotor.set(TalonFXControlMode.PercentOutput, 0.2);
                if (ArmMotor.getStatorCurrent() > 5)
                {
                    ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                    forceChangeState(IntakeState.DEFENCE);
                }
            break;
        }
        
    }

    @Override
    public void runCalibration()
    {
        forceChangeState(IntakeState.CALIBRATING);
    }

    private boolean WithinRange(double value, double r, double g) {return ((value >= r-g)&&(value <= r+g));}

    @Override
    public void updateSmartDashboard()
    {
        SmartDashboard.putString("Intake/Status", intakeStatus.name());
    }
    public void changeState(IntakeState newState)
    {
        if (intakeStatus != IntakeState.CALIBRATING)
        {
            forceChangeState(newState);
        }
    }
    private void forceChangeState(IntakeState newState)
    {
        intakeStatus = newState;
    }
}