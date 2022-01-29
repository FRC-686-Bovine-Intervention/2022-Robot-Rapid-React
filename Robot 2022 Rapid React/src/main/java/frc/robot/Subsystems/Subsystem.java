package frc.robot.Subsystems;

public abstract class Subsystem {
    /**Is called once when the robot starts up*/
    public abstract void init();
    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
}
