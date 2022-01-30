package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Intake subsystem</h4>
 * Has 3 states: <b>Defence</b>, <b>Intake</b>, and <b>Outtake</b><p>
 * <b>Defence</b> is when the intake is resting and the rollers are not spinning<p>
 * <b>Intake</b> is when the intake is down and the rollers are rolling balls in<p>
 * <b>Outtake</b> is when the intake is up and the rollers are rolling balls out<p>
*/
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {if(instance == null){instance = new Intake();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {

    }

    @Override
    public void updateSmartDashboard()
    {
        
    }
}