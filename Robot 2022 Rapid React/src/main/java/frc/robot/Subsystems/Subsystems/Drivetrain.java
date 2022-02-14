package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Drivetrain subsystem</h4>
 * Takes inputs from joystick axes and gives power to drive motors accordingly<p>
 * Should only be enabled when the current or most recent state of the climber is DEFENSE
*/
public class Drivetrain extends Subsystem {
    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {if(instance == null){instance = new Drivetrain();}return instance;}

    public TalonFX LeftMaster, LeftSlave, RightMaster, RightSlave;
    public double LeftPower, RightPower;

    private Drivetrain()
    {
        LeftMaster  = new TalonFX(Constants.kLeftMasterID);
        LeftSlave   = new TalonFX(Constants.kLeftSlaveID);
        RightMaster = new TalonFX(Constants.kRightMasterID);
        RightSlave  = new TalonFX(Constants.kRightSlaveID);

        LeftMaster.setInverted  (false);
        LeftSlave.setInverted   (false);
        RightMaster.setInverted (true);
        RightSlave.setInverted  (true);

        LeftSlave.follow(LeftMaster);
        RightSlave.follow(RightMaster);

        SmartDashboard.putBoolean("Drivetrain/Enabled", true);
    }

    @Override
    public void run(){}

    @Override
    public void runCalibration(){}
    
    @Override
    public void runTestMode(){}

    @Override
    public void updateShuffleboard(){}

    public void setAxis(Vector2d axis) {setPower(axis.y-axis.x, axis.y+axis.x);}

    public void setPower(double leftPower, double rightPower){}
}