package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    }

    @Override
    public void run()
    {
        LeftMaster.set(TalonFXControlMode.PercentOutput, LeftPower);
        RightMaster.set(TalonFXControlMode.PercentOutput, RightPower);
    }

    @Override
    public void runTestMode() {run();}

    @Override
    public void disable()
    {
        LeftMaster.set(TalonFXControlMode.PercentOutput, 0);
        RightMaster.set(TalonFXControlMode.PercentOutput, 0);
    }

    @Override public void runCalibration(){}
    
    public void setAxis(Vector2d axis) {setPower(axis.y-axis.x, axis.y+axis.x);}

    public void setPower(double leftPower, double rightPower)
    {
        LeftPower = leftPower;
        RightPower = rightPower;
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    private NetworkTableEntry LeftMasterCurrent =   tab.add("Left Master Current", 0)   .getEntry();
    private NetworkTableEntry RightMasterCurrent =  tab.add("Right Master Current", 0)  .getEntry();
    private NetworkTableEntry EnabledEntry =        tab.add("Enabled", true)            .getEntry();

    @Override
    public void updateShuffleboard()
    {
        Enabled = EnabledEntry.getBoolean(true);
        LeftMasterCurrent.setDouble(LeftMaster.getStatorCurrent());
        RightMasterCurrent.setDouble(RightMaster.getStatorCurrent());
    }
}