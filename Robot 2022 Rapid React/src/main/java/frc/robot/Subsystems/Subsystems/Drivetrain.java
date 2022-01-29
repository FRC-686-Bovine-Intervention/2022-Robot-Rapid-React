package frc.robot.Subsystems.Subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Drivetrain subsystem</h4>
 * Takes inputs from joystick axis and gives power to drive motors accordingly<p>
 * Should only be enabled when the current or most recent state of the climber is Defence
*/
public class Drivetrain extends Subsystem {
    private VictorSPX leftMaster;
    private VictorSPX rightMaster;
    private VictorSPX leftSlave;
    private VictorSPX rightSlave;
    private Joystick thrustmaster;
    private static Drivetrain instance = null;

    public Drivetrain()
    {
        leftMaster =    new VictorSPX(0);
        leftSlave =     new VictorSPX(1);
        rightMaster =   new VictorSPX(2);
        rightSlave =    new VictorSPX(3);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(InvertType.FollowMaster);
    }

    @Override
    public Drivetrain getInstance() {if(instance == null){instance = new Drivetrain();}return instance;}

    @Override
    public void init()
    {
        thrustmaster = new Joystick(0);
    }
    @Override
    public void run()
    {
        double leftpower = -thrustmaster.getY()+thrustmaster.getX();
        double rightpower = -thrustmaster.getY()-thrustmaster.getX();
        leftMaster.set(VictorSPXControlMode.PercentOutput, leftpower);
        rightMaster.set(VictorSPXControlMode.PercentOutput, rightpower);
    }
}