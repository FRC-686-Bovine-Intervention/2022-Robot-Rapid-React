package frc.robot.Subsystems;

public abstract class Subsystem {
    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
    /**Put all the variables you want to project to SmartDashboard here */
    public abstract void updateSmartDashboard();
    /**Put all calibration code here */
    public abstract void runCalibration();
}
