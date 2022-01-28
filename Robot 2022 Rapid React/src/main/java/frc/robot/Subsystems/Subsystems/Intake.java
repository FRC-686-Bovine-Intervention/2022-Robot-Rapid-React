package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**Contains all code for the Intake subsystem*/
public class Intake extends Subsystem {
    private static Intake instance = null;
    @Override
    public Intake getInstance() {if(instance == null){instance = new Intake();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {

    }
}