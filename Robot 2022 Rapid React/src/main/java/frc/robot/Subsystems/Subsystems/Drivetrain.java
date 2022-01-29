package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**<h4>Contains all code for the Drivetrain subsystem</h4>
 * Takes inputs from joystick axis and gives power to drive motors accordingly<p>
 * Should only be enabled when the current or most recent state of the climber is Defence
*/
public class Drivetrain extends Subsystem {
    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {if(instance == null){instance = new Drivetrain();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {
        
    }
}