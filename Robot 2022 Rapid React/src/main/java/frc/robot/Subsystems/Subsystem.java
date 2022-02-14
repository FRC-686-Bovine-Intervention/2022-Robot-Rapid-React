package frc.robot.Subsystems;

public abstract class Subsystem {
    // private static SUBNAME instance;
    // public static SUBNAME getInstance() {if (instance == null) {instance == SUBNAME();} return instance}

    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
    /**Put all the variables you want to project to SmartDashboard here */
    public boolean Enabled;
    public abstract void updateShuffleboard();
    public abstract void runTestMode();
    public boolean calibrated = true;
    /**Put all calibration code here */
    public void runCalibration() {calibrated = false;}
}
