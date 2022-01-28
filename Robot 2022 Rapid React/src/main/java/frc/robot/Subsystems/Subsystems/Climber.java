package frc.robot.Subsystems.Subsystems;

import frc.robot.Subsystems.Subsystem;

/**Contains all code for the Climber subsystem*/
public class Climber extends Subsystem {
    private static Climber instance = null;
    @Override
    public Climber getInstance() {if(instance == null){instance = new Climber();}return instance;}

    @Override
    public void init()
    {

    }
    @Override
    public void run()
    {

    }
}
