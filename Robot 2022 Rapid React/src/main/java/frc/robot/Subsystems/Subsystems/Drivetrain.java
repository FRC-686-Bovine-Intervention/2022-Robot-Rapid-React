package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**Contains all code for the Drivetrain subsystem*/
public class Drivetrain extends Subsystem {
    private static Drivetrain instance = null;
    @Override
    public Drivetrain getInstance() {if(instance == null){instance = new Drivetrain();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {

    }
}