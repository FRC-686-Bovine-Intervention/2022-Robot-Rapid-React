package frc.robot.Subsystems;

public abstract class Subsystem {
    // private static SUBNAME instance;
    // public static SUBNAME getInstance() {if (instance == null) {instance == SUBNAME();} return instance}

    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
    /**Put all the variables you want to project to SmartDashboard here */
    public abstract void updateSmartDashboard();

    public boolean calibrated;
    /**Put all calibration code here */
    public abstract void runCalibration();
}
