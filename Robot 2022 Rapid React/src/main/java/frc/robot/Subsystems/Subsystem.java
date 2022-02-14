package frc.robot.Subsystems;

public abstract class Subsystem {
    /*
        private SUBNAME instance;
        public SUBNAME getInstance() {if(instance == null){instance = new SUBNAME();}return instance;}

        private SUBNAME()
        {
            
        }
    */
    
    /**Returns true if the subsystem is supposed to be active, otherwise false*/
    public boolean Enabled;
    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
    /**Is called every tick the subsystem is supposed to be active in test mode*/
    public abstract void runTestMode();
    /**Put all the variables you want to project to Shuffleboard here*/
    public abstract void updateShuffleboard();
    /**Returns true if the subsystem has calibrated in the past */
    public boolean calibrated = true;
    /**Put all calibration code here*/
    public void runCalibration() {calibrated = false;}
}
